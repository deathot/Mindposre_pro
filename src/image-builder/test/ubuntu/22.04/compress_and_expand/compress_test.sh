file_name=${BASH_SOURCE[0]}
path=$(cd "$(dirname "$file_name")" && pwd)
modify_compress_sh() {
  sed -i "s/^if ! main \"\$1\" \"\$2\"; then$/#if ! main \"\$1\" \"\$2\"; then/" "$1"
  sed -i "s/^  log \"sd card compress failed!\"$/  #log \"sd card compress failed!\"/" "$1"
  sed -i "s/^else$/#else/" "$1"
  sed -i "s/^  log \"sd card compress success!\"$/  #log \"sd card compress success!\"/" "$1"
  sed -i "s/^fi/#fi/" "$1"
}

recover_compress_sh() {
  sed -i "s/^#if ! main \"\$1\" \"\$2\"; then$/if ! main \"\$1\" \"\$2\"; then/" "$1"
  sed -i "s/^  #log \"sd card compress failed!\"$/  log \"sd card compress failed!\"/" "$1"
  sed -i "s/^#else$/else/" "$1"
  sed -i "s/^  #log \"sd card compress success!\"$/  log \"sd card compress success!\"/" "$1"
  sed -i "s/^#fi/fi/" "$1"
}

assert_true() {
  if [ "$1" == "$2" ]; then
      return 0
  else
      return 1
  fi
}

test_create_zeroes_image() {
  initial_capacity=102400
  test_img_path="$path"/test.img
  create_zeroes_image "$test_img_path"
  result_expected="52428800"
  result_actual=$(ls -l "$test_img_path" | cut -d' ' -f5)
  if ! assert_true "$result_expected" "$result_actual"; then
      echo "Test failed!"
  else
      echo "Test success!"
  fi
  rm "$test_img_path"
}

test_create_visual_disk() {
  losetup_flag=$(losetup -a | grep /dev/loop0)
  if [ "$losetup_flag" != "" ]; then
      losetup -d /dev/loop0
  fi
  initial_capacity=102400
  test_img_path="$path"/test.img
  create_zeroes_image "$test_img_path"
  create_visual_disk "$test_img_path"
  result_expected="/dev/loop0"
  result_actual=$(fdisk -l | grep "/dev/loop0" | awk -F '[: ]' '{print $2}')
  echo $result_actual
  if ! assert_true "$result_expected" "$result_actual"; then
      echo "Test failed!"
  else
      echo "Test success!"
  fi
  losetup -d "/dev/loop0"
  rm "$test_img_path"
}

test_create_part() {
  losetup_flag=$(losetup -a | grep /dev/loop0)
  if [ "$losetup_flag" != "" ]; then
      losetup -d /dev/loop0
  fi
  initial_capacity=102400
  test_img_path="$path"/test.img
  create_zeroes_image "$test_img_path"
  create_visual_disk "$test_img_path"
  parted -s "$path_visual_disk" mklabel gpt
  create_part "$path_visual_disk" "/dev/loop0p1" "fat" "34" "166"
  result_expected="133"
  result_actual=$(fdisk -l | grep /dev/loop0p1 | cut -d ' ' -f13)
  if ! assert_true "$result_expected" "$result_actual"; then
      echo "Test failed!"
  else
      echo "Test success!"
  fi
  losetup -d "/dev/loop0"
  rm "$test_img_path"
}

main() {
  file_name=${BASH_SOURCE[0]}
  path=$(cd "$(dirname "$file_name")" && pwd)
  target_script_path="$path"/../../../src/compress_and_expand/minimal/compress.sh
  modify_compress_sh "$target_script_path"
  source "$target_script_path"
  test_create_zeroes_image
  test_create_visual_disk
  test_create_part
  recover_compress_sh "$target_script_path"
}

main