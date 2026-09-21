// Emulador do Livox MID-360 para o Gazebo Harmonic.
//
// O gz-sim so tem lidar em grade uniforme. Este no recebe uma grade DENSA renderizada pelo gpu_lidar
// e a amostra nas direcoes do padrao de varredura real do MID-360 (arquivo CSV do plugin de simulacao
// da Livox / CTU-MRS: 800 mil direcoes = 4 s a 200 mil pontos/s; cada quadro de 0,1 s usa as proximas
// 20 mil). O resultado tem a distribuicao angular nao repetitiva do sensor, o tempo de captura de cada
// ponto (5 us por ponto) e o formato do driver livox_ros_driver2 (PointCloud2 XYZRTLT ou CustomMsg).
//
// Distorcao de movimento: o Gazebo renderiza o quadro num unico instante t0, mas o sensor real captura
// o ponto k em t0 + k*5us, com o robo ja em outra pose. Como os pontos saem carimbados com esses tempos,
// um LIO vai "corrigir" a nuvem por eles; se a geometria fosse instantanea, a correcao a estragaria.
// Por isso cada ponto e reexpresso no referencial do sensor no seu instante de captura, usando a pose
// verdadeira do robo (interpolada). O quadro e publicado ao fim da janela de 0,1 s, como no sensor real.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#ifdef HAVE_LIVOX_CUSTOM_MSG
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class Mid360Emulator : public rclcpp::Node {
 public:
  Mid360Emulator() : Node("livox_mid360_emulator") {
    pattern_file_ = declare_parameter<std::string>("pattern_file", "");
    points_per_frame_ = declare_parameter<int>("points_per_frame", 20000);
    point_period_ns_ = declare_parameter<int>("point_period_ns", 5000);  // 200 mil pontos/s
    h_min_ = declare_parameter<double>("h_min", -M_PI);
    h_max_ = declare_parameter<double>("h_max", M_PI);
    v_min_ = declare_parameter<double>("v_min", -0.125838);
    v_max_ = declare_parameter<double>("v_max", 0.910364);
    frame_id_ = declare_parameter<std::string>("frame_id", "livox_frame");
    format_ = declare_parameter<std::string>("format", "pointcloud2");  // pointcloud2 | custom
    motion_distortion_ = declare_parameter<bool>("motion_distortion", true);
    reflectivity_ = declare_parameter<int>("reflectivity", 100);
    time_bins_ = declare_parameter<int>("time_bins", 20);
    const auto xyz = declare_parameter<std::vector<double>>("base_to_sensor_xyz", {0.0, 0.0, 0.301});
    const auto rpy = declare_parameter<std::vector<double>>("base_to_sensor_rpy", {0.0, 0.0, 0.0});
    T_bs_ = Eigen::Translation3d(xyz[0], xyz[1], xyz[2]) * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());

    if (!loadPattern()) throw std::runtime_error("padrao de varredura nao carregado: " + pattern_file_);

    if (format_ == "custom") {
#ifdef HAVE_LIVOX_CUSTOM_MSG
      pub_custom_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar", 10);
#else
      throw std::runtime_error("format:=custom exige compilar com o pacote livox_ros_driver2 no ambiente");
#endif
    } else {
      pub_pc2_ = create_publisher<PointCloud2>("livox/lidar", rclcpp::SensorDataQoS());
    }
    sub_dense_ = create_subscription<PointCloud2>("livox/dense_points", 10,
                                                  [this](PointCloud2::ConstSharedPtr m) { onDense(m); });
    if (motion_distortion_)
      sub_gt_ = create_subscription<nav_msgs::msg::Odometry>("ground_truth/odom", 200,
                                                             [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onTruth(m); });
    RCLCPP_INFO(get_logger(), "MID-360: %zu direcoes, %d pontos/quadro, formato %s, distorcao de movimento %s",
                pattern_az_.size(), points_per_frame_, format_.c_str(), motion_distortion_ ? "ligada" : "desligada");
  }

 private:
  struct Pose { double t; Eigen::Vector3d p; Eigen::Quaterniond q; };

  bool loadPattern() {
    std::ifstream f(pattern_file_);
    if (!f) return false;
    std::string line;
    std::getline(f, line);  // cabecalho: Time/s,Azimuth/deg,Zenith/deg
    double idx, az, zen;
    while (std::getline(f, line)) {
      if (std::sscanf(line.c_str(), "%lf,%lf,%lf", &idx, &az, &zen) != 3) continue;
      pattern_az_.push_back(static_cast<float>(az * M_PI / 180.0));
      pattern_el_.push_back(static_cast<float>((90.0 - zen) * M_PI / 180.0));  // elevacao = 90 - zenite
    }
    return pattern_az_.size() >= static_cast<size_t>(points_per_frame_);
  }

  // indice na grade densa (linha = anel vertical, coluna = azimute) de cada direcao do padrao
  void buildIndex(uint32_t W, uint32_t H) {
    pattern_idx_.resize(pattern_az_.size());
    for (size_t k = 0; k < pattern_az_.size(); ++k) {
      const double c = (pattern_az_[k] - h_min_) / (h_max_ - h_min_) * (W - 1);
      const double r = (pattern_el_[k] - v_min_) / (v_max_ - v_min_) * (H - 1);
      const long ci = std::clamp<long>(std::lround(c), 0, W - 1), ri = std::clamp<long>(std::lround(r), 0, H - 1);
      pattern_idx_[k] = static_cast<uint32_t>(ri * W + ci);
    }
    W_ = W; H_ = H;
    RCLCPP_INFO(get_logger(), "grade densa %ux%u (%.3f x %.3f graus por celula)", W, H,
                (h_max_ - h_min_) / (W - 1) * 180 / M_PI, (v_max_ - v_min_) / (H - 1) * 180 / M_PI);
  }

  void onTruth(nav_msgs::msg::Odometry::ConstSharedPtr m) {
    const auto &p = m->pose.pose.position; const auto &q = m->pose.pose.orientation;
    truth_.push_back({rclcpp::Time(m->header.stamp).seconds(), {p.x, p.y, p.z}, Eigen::Quaterniond(q.w, q.x, q.y, q.z)});
    while (truth_.size() > 400) truth_.pop_front();
    process();
  }

  void onDense(PointCloud2::ConstSharedPtr m) {
    queue_.push_back(m);
    while (queue_.size() > 8) { queue_.pop_front(); RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "fila cheia: quadro descartado"); }
    process();
  }

  bool sensorPose(double t, Eigen::Isometry3d &T) const {  // pose do sensor no mundo em t (interpolada)
    if (truth_.size() < 2 || t < truth_.front().t || t > truth_.back().t) return false;
    auto hi = std::lower_bound(truth_.begin(), truth_.end(), t, [](const Pose &a, double v) { return a.t < v; });
    if (hi == truth_.begin()) ++hi;
    const auto lo = hi - 1;
    const double a = (hi->t > lo->t) ? (t - lo->t) / (hi->t - lo->t) : 0.0;
    Eigen::Isometry3d Twb = Eigen::Isometry3d::Identity();
    Twb.linear() = lo->q.slerp(a, hi->q).toRotationMatrix();
    Twb.translation() = (1 - a) * lo->p + a * hi->p;
    T = Twb * T_bs_;
    return true;
  }

  void process() {
    while (!queue_.empty()) {
      const auto m = queue_.front();
      const double t0 = rclcpp::Time(m->header.stamp).seconds();
      const double t_end = t0 + points_per_frame_ * point_period_ns_ * 1e-9;
      bool distort = motion_distortion_;
      if (distort) {
        if (truth_.empty() || truth_.back().t < t_end) {          // ainda falta pose verdadeira ate o fim do quadro
          if (queue_.size() < 5) return;                          // espera; se acumular, segue sem distorcao
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "sem /ground_truth/odom: publicando sem distorcao de movimento");
          distort = false;
        }
      }
      emit(*m, t0, distort);
      queue_.pop_front();
    }
  }

  void emit(const PointCloud2 &m, double t0, bool distort) {
    int ox = -1, oy = -1, oz = -1;
    for (const auto &f : m.fields) {
      if (f.datatype != PointField::FLOAT32) continue;
      if (f.name == "x") ox = f.offset; else if (f.name == "y") oy = f.offset; else if (f.name == "z") oz = f.offset;
    }
    if (ox < 0 || oy < 0 || oz < 0) { RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "nuvem densa sem campos x/y/z float32"); return; }
    if (m.width != W_ || m.height != H_) buildIndex(m.width, m.height);
    if (!checked_) selfCheck(m, ox, oy, oz);

    // transformadas por fatia de tempo: referencial do sensor em t0 -> referencial do sensor em t_k
    std::vector<Eigen::Isometry3d> rel(time_bins_, Eigen::Isometry3d::Identity());
    Eigen::Isometry3d T0;
    if (distort && sensorPose(t0, T0)) {
      const double bin_dt = points_per_frame_ * point_period_ns_ * 1e-9 / time_bins_;
      for (int b = 0; b < time_bins_; ++b) {
        Eigen::Isometry3d Tk;
        if (sensorPose(t0 + (b + 0.5) * bin_dt, Tk)) rel[b] = Tk.inverse() * T0;
      }
    } else {
      distort = false;
    }

    const size_t start = (frame_count_ * static_cast<size_t>(points_per_frame_)) % pattern_idx_.size();
    ++frame_count_;
    const int64_t t0_ns = rclcpp::Time(m.header.stamp).nanoseconds();
    struct Out { float x, y, z; uint32_t k; };
    std::vector<Out> out; out.reserve(points_per_frame_);
    const int per_bin = std::max(1, points_per_frame_ / time_bins_);
    for (int k = 0; k < points_per_frame_; ++k) {
      const uint32_t idx = pattern_idx_[(start + k) % pattern_idx_.size()];
      const uint8_t *p = m.data.data() + static_cast<size_t>(idx) * m.point_step;
      float x, y, z;
      std::memcpy(&x, p + ox, 4); std::memcpy(&y, p + oy, 4); std::memcpy(&z, p + oz, 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;  // sem retorno
      if (distort) {
        const Eigen::Vector3d q = rel[std::min(k / per_bin, time_bins_ - 1)] * Eigen::Vector3d(x, y, z);
        x = q.x(); y = q.y(); z = q.z();
      }
      out.push_back({x, y, z, static_cast<uint32_t>(k)});
    }

    if (pub_pc2_) {                       // PointCloud2 no layout do livox_ros_driver2 (xfer_format 0, PointXYZRTLT)
      PointCloud2 c;
      c.header.stamp = m.header.stamp; c.header.frame_id = frame_id_;
      c.height = 1; c.width = out.size(); c.is_bigendian = false; c.is_dense = true; c.point_step = 26; c.row_step = c.point_step * c.width;
      auto field = [](const char *n, uint32_t off, uint8_t dt) { PointField f; f.name = n; f.offset = off; f.datatype = dt; f.count = 1; return f; };
      c.fields = {field("x", 0, PointField::FLOAT32), field("y", 4, PointField::FLOAT32), field("z", 8, PointField::FLOAT32),
                  field("intensity", 12, PointField::FLOAT32), field("tag", 16, PointField::UINT8), field("line", 17, PointField::UINT8),
                  field("timestamp", 18, PointField::FLOAT64)};
      c.data.resize(c.row_step);
      const float inten = static_cast<float>(reflectivity_);
      for (size_t i = 0; i < out.size(); ++i) {
        uint8_t *d = c.data.data() + i * 26;
        const uint8_t tag = 0x00, line = out[i].k % 4;
        const double ts = static_cast<double>(t0_ns + static_cast<int64_t>(out[i].k) * point_period_ns_);  // ns absolutos
        std::memcpy(d, &out[i].x, 4); std::memcpy(d + 4, &out[i].y, 4); std::memcpy(d + 8, &out[i].z, 4);
        std::memcpy(d + 12, &inten, 4); d[16] = tag; d[17] = line; std::memcpy(d + 18, &ts, 8);
      }
      pub_pc2_->publish(c);
    }
#ifdef HAVE_LIVOX_CUSTOM_MSG
    if (pub_custom_) {
      livox_ros_driver2::msg::CustomMsg c;
      c.header.stamp = m.header.stamp; c.header.frame_id = frame_id_;
      c.timebase = static_cast<uint64_t>(t0_ns); c.point_num = out.size(); c.lidar_id = 0;
      c.points.resize(out.size());
      for (size_t i = 0; i < out.size(); ++i) {
        auto &pt = c.points[i];
        pt.offset_time = out[i].k * static_cast<uint32_t>(point_period_ns_);
        pt.x = out[i].x; pt.y = out[i].y; pt.z = out[i].z;
        pt.reflectivity = static_cast<uint8_t>(reflectivity_); pt.tag = 0x10; pt.line = out[i].k % 4;
      }
      pub_custom_->publish(c);
    }
#endif
  }

  // confere, no primeiro quadro, se a ordem da grade do gz e a que o indice assume
  void selfCheck(const PointCloud2 &m, int ox, int oy, int oz) {
    double worst_az = 0, worst_el = 0; size_t n = 0;
    for (size_t i = 0; i < static_cast<size_t>(m.width) * m.height; i += 97) {
      const uint8_t *p = m.data.data() + i * m.point_step;
      float x, y, z; std::memcpy(&x, p + ox, 4); std::memcpy(&y, p + oy, 4); std::memcpy(&z, p + oz, 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || std::hypot(x, y) < 0.5) continue;
      const double az = h_min_ + (i % m.width) * (h_max_ - h_min_) / (m.width - 1);
      const double el = v_min_ + (i / m.width) * (v_max_ - v_min_) / (m.height - 1);
      double daz = std::atan2(y, x) - az; daz = std::atan2(std::sin(daz), std::cos(daz));
      worst_az = std::max(worst_az, std::abs(daz)); worst_el = std::max(worst_el, std::abs(std::atan2(z, std::hypot(x, y)) - el)); ++n;
    }
    if (n < 50) return;
    checked_ = true;
    const double cell = (h_max_ - h_min_) / (m.width - 1);
    if (worst_az > cell || worst_el > cell)
      RCLCPP_ERROR(get_logger(), "a grade densa NAO tem a ordem esperada (erro az %.3f, el %.3f graus): confira os angulos do sensor",
                   worst_az * 180 / M_PI, worst_el * 180 / M_PI);
    else
      RCLCPP_INFO(get_logger(), "ordem da grade conferida em %zu pontos (erro max az %.4f, el %.4f graus)", n, worst_az * 180 / M_PI, worst_el * 180 / M_PI);
  }

  std::string pattern_file_, frame_id_, format_;
  int points_per_frame_, point_period_ns_, reflectivity_, time_bins_;
  double h_min_, h_max_, v_min_, v_max_;
  bool motion_distortion_, checked_ = false;
  uint32_t W_ = 0, H_ = 0;
  size_t frame_count_ = 0;
  Eigen::Isometry3d T_bs_;
  std::vector<float> pattern_az_, pattern_el_;
  std::vector<uint32_t> pattern_idx_;
  std::deque<Pose> truth_;
  std::deque<PointCloud2::ConstSharedPtr> queue_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pc2_;
#ifdef HAVE_LIVOX_CUSTOM_MSG
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_custom_;
#endif
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_dense_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gt_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Mid360Emulator>()); }
  catch (const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("livox_mid360_emulator"), "%s", e.what()); return 1; }
  rclcpp::shutdown();
  return 0;
}
