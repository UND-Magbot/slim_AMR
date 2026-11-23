#!/usr/bin/env bash
# 관리자 화면(api_node)이 네트워크를 설정할 수 있게 권한을 연다.
#
# 왜 필요한가
#   api_node 는 und_amr1 로 돈다. NetworkManager 는 polkit 으로 권한을
#   가르는데, 로그인 세션이 아닌 서비스에는 기본적으로 쓰기를 막는다.
#   주변 WiFi 를 다시 훑는 것(rescan)조차 막히므로, 화면에 붙어 있는
#   AP 하나만 보인다.
#
# 무엇을 여는가
#   org.freedesktop.NetworkManager.* 만 연다. 다른 권한은 건드리지 않는다.
#
#   sudo ~/projects/UND_AMR/tools/allow_network_config.sh
set -e
USER_NAME="${1:-und_amr1}"
if [ "$(id -u)" -ne 0 ]; then
  echo "루트로 실행하세요:  sudo $0 $USER_NAME" >&2
  exit 1
fi

# polkit 은 두 세대가 있다. 0.106 이상은 자바스크립트 rules.d 를 쓰고,
# 0.105(우분투 22.04)는 .pkla 파일을 쓴다. 있는 쪽에 맞춰 쓴다.
if [ -d /etc/polkit-1/rules.d ]; then
  RULE=/etc/polkit-1/rules.d/50-und-amr-network.rules
  cat > "$RULE" <<RULE
// UND_AMR 관리자 화면이 네트워크를 설정할 수 있게 한다.
// NetworkManager 관련 동작만 열고, 다른 권한은 건드리지 않는다.
polkit.addRule(function(action, subject) {
  if (action.id.indexOf("org.freedesktop.NetworkManager.") === 0 &&
      subject.user === "$USER_NAME") {
    return polkit.Result.YES;
  }
});
RULE
else
  mkdir -p /etc/polkit-1/localauthority/50-local.d
  RULE=/etc/polkit-1/localauthority/50-local.d/50-und-amr-network.pkla
  cat > "$RULE" <<RULE
[UND_AMR admin web may manage NetworkManager]
Identity=unix-user:$USER_NAME
Action=org.freedesktop.NetworkManager.*
ResultAny=yes
ResultInactive=yes
ResultActive=yes
RULE
fi
chmod 644 "$RULE"
echo "규칙 작성: $RULE  (대상 사용자: $USER_NAME)"

systemctl restart polkit
sleep 2
echo "polkit 재시작 완료"

echo
echo "확인 (아래가 오류 없이 나오면 성공):"
sudo -u "$USER_NAME" nmcli dev wifi rescan && echo "  rescan OK" || echo "  rescan 실패 — 로그를 확인하세요"
