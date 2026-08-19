#!/usr/bin/env bash
# Phase-0 load generator for the multicore evaluation: reproduces the
# web-UI/NMOS polling load against the node while `perf cpu` and `spibus`
# sample on the Zephyr shell.
#
# Usage: webload.sh <host> [clients] [interval_s] [duration_s]
#   clients   parallel poll loops (default 4 — matches a few open UI tabs)
#   interval  delay between requests per client (default 0.1 s, the old
#             100 ms UI refresh that triggered the display freeze)
#   duration  total runtime (default 60 s)
#
# Measurement procedure (on the Zephyr shell, PTP follower + SAP active):
#   uart:~$ perf cpu     # opens the window
#   uart:~$ spibus       # opens the bus window
#   ... run this script ...
#   uart:~$ perf cpu     # per-thread CPU share over the load window
#   uart:~$ spibus       # bus held / wire busy / wait times
set -u

HOST=${1:?usage: webload.sh <host> [clients] [interval_s] [duration_s]}
CLIENTS=${2:-4}
INTERVAL=${3:-0.1}
DURATION=${4:-60}

# The endpoints the web UI + an NMOS registry actually poll.
URLS=(
    /api/summary
    /api/streams/rx
    /api/streams/tx
    /api/streams/discovered
    /api/ptp
    /api/network
    /x-nmos/node/v1.3/self
)

end=$((SECONDS + DURATION))
total=0
fails=0

client() {
    local n=0 f=0
    while ((SECONDS < end)); do
        for u in "${URLS[@]}"; do
            if ! curl -sf -m 2 -o /dev/null "http://${HOST}${u}"; then
                ((f++))
            fi
            ((n++))
            sleep "$INTERVAL"
        done
    done
    echo "$n $f"
}

echo "load: ${CLIENTS} clients, ${INTERVAL}s interval, ${DURATION}s against ${HOST}" >&2

pids=()
results=$(mktemp)
trap 'kill "${pids[@]}" 2>/dev/null; rm -f "$results"' EXIT

for ((c = 0; c < CLIENTS; c++)); do
    client >>"$results" &
    pids+=($!)
done
wait "${pids[@]}" 2>/dev/null

while read -r n f; do
    ((total += n, fails += f))
done <"$results"

echo "done: ${total} requests, ${fails} failed ($(awk -v t="$total" -v d="$DURATION" 'BEGIN{printf "%.1f", t/d}') req/s)" >&2
[[ $fails -eq 0 ]]
