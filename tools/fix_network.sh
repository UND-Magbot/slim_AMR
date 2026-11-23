#!/usr/bin/env bash
# 라이다 네트워크가 원격 접속을 방해하지 못하게 고정한다.
#
#   bash tools/fix_network.sh          # 점검 + 수정
#   bash tools/fix_network.sh --check  # 점검만 (아무것도 바꾸지 않음)
#
# 배경
#   라이다는 전용 대역(192.168.158.x)을 쓰는 별도 유선 링크다. 이 링크가
#   기본 경로나 DNS 를 가져가면 인터넷과 원격 접속이 끊긴다. 그리고
#   ufw 를 켤 때 SSH 허용 규칙이 없으면 그 순간 원격이 잘린다.
#   두 가지를 모두 고정해 둔다.

set -u
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

LIDAR_CON="lidar"
WEB_PORT=${WEB_PORT:-8090}      # 관리자 웹 포트 (api_node)
LIDAR_NET="192.168.158.0/24"

say(){ echo; echo "===== $* ====="; }
ok(){  echo "  ✅ $*"; }
no(){  echo "  ❌ $*"; }
warn(){ echo "  ⚠  $*"; }

say "1. 현재 라우팅"
ip route | sed 's/^/  /'
DEF_DEV=$(ip route show default | awk '{print $5; exit}')
if [ -n "$DEF_DEV" ]; then
  LIDAR_DEV=$(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null \
              | awk -F: -v n="$LIDAR_CON" '$1==n{print $2}')
  if [ -n "$LIDAR_DEV" ] && [ "$DEF_DEV" = "$LIDAR_DEV" ]; then
    no "기본 경로가 라이다 링크($DEF_DEV)로 잡혀 있습니다 — 인터넷이 끊깁니다"
  else
    ok "기본 경로: $DEF_DEV (라이다 링크 아님)"
  fi
else
  no "기본 경로가 없습니다"
fi

say "2. 대역 충돌 확인"
# 라이다 대역이 사내망과 겹치면 원격 접속이 그쪽으로 새어나간다
CONFLICT=0
while read -r net dev; do
  [ "$net" = "$LIDAR_NET" ] && continue
  case "$net" in 192.168.158.*) CONFLICT=1; warn "겹치는 대역: $net ($dev)";; esac
done < <(ip -o -4 route show scope link | awk '{print $1, $3}')
[ $CONFLICT -eq 0 ] && ok "라이다 대역($LIDAR_NET)이 다른 망과 겹치지 않습니다"

say "3. 방화벽 (SSH · 관리자 웹)"
if ! command -v ufw >/dev/null; then
  ok "ufw 미설치 — 해당 없음"
elif ! sudo ufw status >/dev/null 2>&1; then
  warn "ufw 상태 조회 실패"
else
  UFW=$(sudo ufw status verbose)
  echo "$UFW" | sed 's/^/  /'
  if echo "$UFW" | grep -q "Status: active"; then
    if echo "$UFW" | grep -qE "^22[/ ]|OpenSSH"; then
      ok "SSH 허용 규칙 있음"
    else
      no "ufw 가 켜져 있는데 SSH 허용 규칙이 없습니다 — 원격이 끊깁니다"
      if [ $CHECK_ONLY -eq 0 ]; then
        sudo ufw allow 22/tcp comment "SSH remote access"
        ok "SSH(22/tcp) 허용 규칙 추가"
      fi
    fi
    # 관리자 웹(8090). ufw 는 막을 때 거부가 아니라 '조용히 버리기' 라
    # 브라우저에서는 그냥 한참 매달리다 끝난다. 서버는 멀쩡한데 왜 안
    # 되는지 알기 어려워, 다른 PC 에서 접속이 안 되면 여기부터 본다.
    # (2026-08-28 실제로 이것 때문에 밖에서 접속이 안 됐다)
    if echo "$UFW" | grep -qE "^${WEB_PORT}[/ ]"; then
      ok "관리자 웹(${WEB_PORT}/tcp) 허용 규칙 있음"
    else
      no "ufw 가 켜져 있는데 ${WEB_PORT} 허용 규칙이 없습니다 — 다른 PC 에서 관리자 화면에 못 들어옵니다"
      if [ $CHECK_ONLY -eq 0 ]; then
        sudo ufw allow ${WEB_PORT}/tcp comment "AMR admin web"
        ok "관리자 웹(${WEB_PORT}/tcp) 허용 규칙 추가"
      fi
    fi
  else
    ok "ufw 비활성 — 차단 없음"
  fi
fi

say "4. 라이다 연결 설정 고정"
if ! nmcli con show "$LIDAR_CON" >/dev/null 2>&1; then
  warn "'$LIDAR_CON' 연결이 없습니다. 먼저 아래로 만드세요:"
  echo "     sudo nmcli con add type ethernet ifname enp2s0 \\"
  echo "       con-name $LIDAR_CON ip4 192.168.158.15/24"
else
  nmcli -f ipv4.never-default,ipv4.dns-priority,ipv4.route-metric,\
ipv4.gateway,connection.autoconnect-priority con show "$LIDAR_CON" \
    2>/dev/null | sed 's/^/  현재: /'
  if [ $CHECK_ONLY -eq 0 ]; then
    sudo nmcli con mod "$LIDAR_CON" \
      ipv4.never-default yes \
      ipv4.gateway "" \
      ipv4.dns "" \
      ipv4.dns-priority 200 \
      ipv4.route-metric 900 \
      ipv4.link-local disabled \
      ipv4.may-fail yes \
      ipv6.method disabled \
      connection.autoconnect yes \
      connection.autoconnect-priority -10
    ok "never-default / DNS 미개입 / 낮은 우선순위로 고정"
    sudo nmcli con up "$LIDAR_CON" >/dev/null 2>&1 && ok "재적용 완료"
  fi
fi

say "5. WiFi 우선순위"
WIFI=$(nmcli -t -f NAME,TYPE con show 2>/dev/null \
       | awk -F: '$2=="802-11-wireless"{print $1; exit}')
if [ -n "$WIFI" ]; then
  echo "  WiFi 연결: $WIFI"
  if [ $CHECK_ONLY -eq 0 ]; then
    sudo nmcli con mod "$WIFI" connection.autoconnect yes \
      connection.autoconnect-priority 10 ipv4.route-metric 100
    ok "WiFi 를 기본 경로 우선으로 고정"
  fi
else
  warn "WiFi 연결을 찾지 못했습니다"
fi

say "6. 적용 후 라우팅"
sleep 1
ip route | sed 's/^/  /'
DEF_DEV2=$(ip route show default | awk '{print $5; exit}')
echo
if [ -n "$DEF_DEV2" ]; then
  ok "기본 경로: $DEF_DEV2"
else
  no "기본 경로 없음 — 확인 필요"
fi
echo "  원격 접속 주소: $(ip -4 -o addr show "${DEF_DEV2:-lo}" \
   | awk '{print $4}' | cut -d/ -f1)"
