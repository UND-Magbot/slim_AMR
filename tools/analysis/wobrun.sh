S=$1
waitdone() { sleep 3; until [ "$(curl -s localhost:8090/route/status | python3 -c 'import sys,json;print(json.load(sys.stdin)["busy"])' 2>/dev/null)" = "False" ]; do sleep 5; done; curl -s localhost:8090/route/status | python3 -c 'import sys,json;d=json.load(sys.stdin);print(d["state"],d["lap"],d["message"])'; }
echo start $(date +%T)
curl -s -X POST localhost:8090/route/start -H 'Content-Type: application/json' -d '{"a":"1","b":"2","repeat":3,"dwell":4,"lift":false}' >/dev/null; waitdone
curl -s -X POST localhost:8090/route/start -H 'Content-Type: application/json' -d '{"a":"1","b":"충전소1","repeat":3,"dwell":4,"lift":false}' >/dev/null; waitdone
echo end $(date +%T)
