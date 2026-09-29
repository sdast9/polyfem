#!/opt/homebrew/bin/bash
# usage: queue2.sh JOBFILE PARALLEL ; same job-line format as queue.sh, no xargs line limit
cd /Users/stevenabramowitch/Downloads/fable_polyfem/default-controller-work
run_one() {
  read -r label scene arm steps threads real to extra <<< "$1"
  [ -f runs/$label/row.json ] && { echo "skip $label"; return; }
  eval "python3 dca_run.py $label --scene $scene --arm $arm --steps $steps --threads $threads --realization $real --timeout ${to:-7200} $extra" >> logs/$label.out 2>&1
  echo "$(date +%H:%M:%S) done $label $(tail -1 logs/$label.out)"
}
while IFS= read -r line; do
  [[ -z "$line" || "$line" == \#* ]] && continue
  while [ "$(jobs -rp | wc -l)" -ge "$2" ]; do wait -n; done
  run_one "$line" &
done < "$1"
wait
