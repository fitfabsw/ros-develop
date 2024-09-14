#!/usr/bin/env bash

# 確定平台
platform=$(uname)

# 根據平台設置 getopt 二進制文件路徑
if [[ $platform == "Darwin" ]]; then
  # macOS
  getopt_bin="/usr/local/opt/gnu-getopt/bin/getopt"
else
  # 假定為 Linux/Ubuntu
  getopt_bin="getopt"
fi

function print_usage {
  echo "Usage: $0 [OPTIONS]"
  echo ""
  echo "Options:"
  max_len=0
  # Store keys in an array
  keys=("${!arg_desc[@]}")

  # Sort keys alphabetically
  IFS=$'\n' sorted_keys=($(sort <<<"${keys[*]}"))
  unset IFS

  # Calculate the maximum length of option keys for formatting
  for key in "${sorted_keys[@]}"; do
    len=${#key}
    if ((len > max_len)); then
      max_len=$len
    fi
  done

  # Print each sorted option key-value pair, aligned
  for key in "${sorted_keys[@]}"; do
    printf "  %-${max_len}s    %s\n" "$key" "${arg_desc[$key]}"
  done
}

function parse_args {
  local OPTIONS=""
  local LONGOPTIONS=""

  for key in "${!arg_desc[@]}"; do
    short_opt=$(echo "$key" | cut -d, -f1)
    long_opt=$(echo "$key" | cut -d, -f2)
    flag_info=${arg_desc[$key]}
    # echo "flag_info:" $flag_info
    if [[ $key == "-h,--help" ]]; then
      # Skip adding help to options as it's already added
      OPTIONS+="${short_opt:1:1}"
      LONGOPTIONS+="${long_opt:2},"
      continue
    # elif [[ $flag_info == *"flag"* ]]; then
    elif [[ $flag_info == *"default: false"* || $flag_info == *"default: true"* ]]; then
      # 如果是布林標誌，不加 ":"
      OPTIONS+="${short_opt:1:1}"
      LONGOPTIONS+="${long_opt:2},"
    else
      OPTIONS+="${short_opt:1:1}:"
      LONGOPTIONS+="${long_opt:2}:,"
    fi
  done

  local PARSED
  PARSED=$($getopt_bin --options="$OPTIONS" --longoptions="${LONGOPTIONS%,}" --name "$0" -- "$@")

  if [[ $? -ne 0 ]]; then
    print_usage
    exit 1
  fi

  eval set -- "$PARSED"

  while true; do
    case "$1" in
    -h | --help)
      print_usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    *)
      for key in "${!arg_desc[@]}"; do
        short_opt=$(echo "$key" | cut -d, -f1)
        long_opt=$(echo "$key" | cut -d, -f2)
        flag_info=${arg_desc[$key]}
        if [[ "$1" == "$short_opt" || "$1" == "$long_opt" ]]; then
          var_name=$(echo "$long_opt" | sed 's/--//;s/-/_/g')
          # if [[ $flag_info == *"flag"* ]]; then
          if [[ $flag_info == *"default: false"* || $flag_info == *"default: true"* ]]; then
            parsed_args["$var_name"]=true
            shift
          else
            parsed_args["$var_name"]=$(echo "$2" | sed 's/^=//')
            shift 2
          fi
          break
        fi
      done
      ;;
    esac
  done
  # 处理位置参数
  positional_args=() # 初始化一个数组来存储位置参数
  while [[ $# -gt 0 ]]; do
    positional_args+=("$1") # 将剩余的参数添加到数组中
    shift                   # 移到下一个参数
  done
}

function parse_flag {
  flagname=$1
  if [[ ${parsed_args[$flagname]} == true ]]; then
    if [[ ${default_flags["--${flagname}"]} == true ]]; then
      result=false
    else
      result=true
    fi
  else
    if [[ ${default_flags["--${flagname}"]} == true ]]; then
      result=true
    else
      result=false
    fi
  fi
  echo $result
}
