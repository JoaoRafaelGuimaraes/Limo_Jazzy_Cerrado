#!/usr/bin/env bash
# Baixa o padrao de varredura real do Livox MID-360 (800 mil direcoes, 25 MB) usado pelo emulador.
# Origem: plugin de simulacao do CTU-MRS, derivado do livox_laser_simulation da Livox (MIT).
# O arquivo nao e versionado neste repositorio: o fork de onde ele vem nao declara licenca.
set -euo pipefail
dest="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)/config/livox/mid360-real-centr.csv"
url="https://raw.githubusercontent.com/ctu-mrs/Mid360_simulation_plugin/refactoring/livox_laser_simulation/scan_mode/mid360-real-centr.csv"
if [ -s "$dest" ]; then echo "padrao do MID-360 ja presente: $dest"; exit 0; fi
mkdir -p "$(dirname "$dest")"
echo "baixando o padrao do MID-360..."
curl -fL --retry 3 -o "$dest.part" "$url" && mv "$dest.part" "$dest"
echo "ok: $dest ($(wc -l < "$dest") linhas)"
