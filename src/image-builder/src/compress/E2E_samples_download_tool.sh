#!/usr/bin/env bash

# 日志函数
log() {
  cur_date=$(date "+%Y-%m-%d %H:%M:%S")
  echo "[$cur_date] [E2E SAMPLES DOWNLOAD] ""$1"
}

USAGE="Usage: bash $(basename "$0") [-f] -d \"destination\" -s \"source repository\" [-b] \"branch\" \"target_directory_1\" \"target_directory_2\" ..... .
-f           force to update your destination from remote, using git reset hard underlying,
-d           path you want the repository to be in, and the directory will be like destination/target_directory,
-s           source repository which you want to pull, using git remote add underlying,
-b           branch you want to pull from source repository."

while getopts "hfd:s:b" opt; do
  case "$opt" in
    h)
      printf "%s\\n" "$USAGE"
      exit 2
      ;;
    f)
      FORCE=true ;;
    d)
      DESTINATION="$OPTARG"
      if [ "$DESTINATION" == "" ]; then
        log "Please provide your download destination path."
        exit 1
      fi
      ;;
    s)
      SOURCE_REPOSITORY="$OPTARG"
      if [ "$SOURCE_REPOSITORY" == "" ]; then
        log "Please provide source repository."
        exit 1
      fi
      ;;
    b)
      BRANCH="$OPTARG"
      ;;
    ?)
      printf "ERROR: did not recognize option '%s', please try -h\\n" "$opt"
      exit 1
      ;;
  esac
done
shift "$((OPTIND-1))"

main() {
  log "Download directory $* from $SOURCE_REPOSITORY to $DESTINATION"
  mkdir -p "$DESTINATION"
  cd "$DESTINATION" || exit 1 > /dev/null 2>&1
  git init > /dev/null 2>&1
  git remote add -f origin "$SOURCE_REPOSITORY" > /dev/null 2>&1
  git config core.sparsecheckout true
  touch .git/info/sparse-checkout
  if [ "$FORCE" == "true" ]; then
    cat /dev/null > .git/info/sparse-checkout
    for directory in "$@"; do
      printf "%s\n" "$directory" >> .git/info/sparse-checkout
    done
  else
    for directory in "$@"; do
      if [ "$(cat < .git/info/sparse-checkout | grep "$directory")" == "" ]; then
        printf "%s\n" "$directory" >> .git/info/sparse-checkout
      fi
    done
  fi
  git read-tree -mu HEAD > /dev/null 2>&1
  if [ "$FORCE" == "true" ]; then
    if [ "$BRANCH" == "" ]; then
      git reset --hard origin/master
    else
      git reset --hard origin/"$BRANCH"

    fi
  else
    if [ "$BRANCH" == "" ]; then
      git pull origin master
    else
      git pull origin "$BRANCH"
    fi
  fi
}

if ! main "$@"; then
  log "Download E2E samples failed!"
else
  log "Download E2E samples successfully!"
fi
