modify_expand_sh() {
  sed -i "s/^main$/#main/" "$1"
}

recover_expand_sh() {
  sed -i "s/^#main$/main/" "$1"
}

assert_true() {
  if [ "$1" == "$2" ]; then
    return 0
  else
    return 1
  fi
}

test_calculate_parts_capacities() {
  dev_name="$1"
  result_expected=1
  calculate_parts_capacities
  result_actual=$?
  if ! assert_true "$result_expected" "$result_actual"; then
      echo "Test failed!"
  else
      echo "Test success!"
  fi
}

test_create_part() {
  losetup_flag=$(losetup -a | grep /dev/loop0)
  if [ "$losetup_flag" != "" ]; then
      losetup -d /dev/loop0
  fi
  test_img_path=test.img
  fallocate -l 102400 "$test_img_path"
  fallocate -z -l 102400 "$test_img_path"
  path_visual_disk=$(losetup --show -Pvf test.img)
  parted -s "$path_visual_disk" mklabel gpt
  create_part "$path_visual_disk" "$path_visual_disk"p1 "fat" "34" "166"
  result_expected="133"
  result_actual=$(fdisk -l | grep "$path_visual_disk"p1 | cut -d ' ' -f13)
  if ! assert_true "$result_expected" "$result_actual"; then
      echo "Test failed!"
  else
      echo "Test success!"
  fi
  losetup -d "$path_visual_disk"
  rm "$test_img_path"
}

test_add_label() {
  losetup_flag=$(losetup -a | grep /dev/loop0)
  if [ "$losetup_flag" != "" ]; then
      losetup -d /dev/loop0
  fi
  test_img_path=test.img
  fallocate -l 102400 "$test_img_path"
  fallocate -z -l 102400 "$test_img_path"
  path_visual_disk=$(losetup --show -Pvf test.img)
  parted -s "$path_visual_disk" mklabel gpt
  create_part "$path_visual_disk" "$path_visual_disk"p1 "fat" "34" "166"
  make_fs "$path_visual_disk"p1 "fat"
  add_label "$path_visual_disk"p1 "test"
  result_expected=test
  result_actual="$(blkid | grep "$path_visual_disk"p1 | awk '{ delete vars; for(i = 1; i <= NF; ++i) { n = index($i, "="); if(n) { vars[substr($i, 1, n - 1)] = substr($i, n + 1) } } Var = vars["Var"] } { print vars["LABEL"] }' | cut -d'"' -f2)"
  if ! assert_true "$result_expected" "$result_actual"; then
      echo "Test failed!"
  else
      echo "Test success!"
  fi
  losetup -d "$path_visual_disk"
  rm "$test_img_path"
}

test_resize_part() {
  losetup_flag=$(losetup -a | grep /dev/loop0)
  if [ "$losetup_flag" != "" ]; then
      losetup -d /dev/loop0
  fi
  test_img_path=test.img
  fallocate -l 102400 "$test_img_path"
  fallocate -z -l 102400 "$test_img_path"
  path_visual_disk=$(losetup --show -Pvf test.img)
  dev_name="$path_visual_disk"
  parted -s "$path_visual_disk" mklabel gpt
  create_part "$path_visual_disk" "$path_visual_disk"p1 "fat" "34" "100"
  resize_part "$path_visual_disk"p1 "166"
  result_expected="133"
  result_actual=$(fdisk -l | grep "$path_visual_disk"p1  | cut -d ' ' -f13)
  if ! assert_true "$result_expected" "$result_actual"; then
      echo "Test failed!"
  else
      echo "Test success!"
  fi
  losetup -d "$path_visual_disk"
  rm "$test_img_path"
}

main() {
  file_name=${BASH_SOURCE[0]}
  path=$(cd "$(dirname "$file_name")" && pwd)
  target_script_path="$path"/../../../src/compress_and_expand/minimal/expand.sh
  modify_expand_sh "$target_script_path"
  source "$target_script_path"
  test_calculate_parts_capacities "test"
  test_create_part
  test_add_label
  test_resize_part
  recover_expand_sh "$target_script_path"
}

main