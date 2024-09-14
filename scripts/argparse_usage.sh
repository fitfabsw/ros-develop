#!/usr/bin/env bash

source argparse.sh

# 選項描述
declare -A arg_desc=(
  ["-u,--ubuntu-codename"]="Ubuntu codename (default: focal)"
  ["-r,--rosdistro"]="ROS distribution (default: noetic)"
  ["-w,--workspace"]="Workspace name (default: my_ws)"
  ["-a,--appendbashrc"]="append bashrc (flag)"
  ["-d,--disable-convert"]="disable convert (flag)"
  ["-x,--excludes"]="excluded packages"
  ["-h,--help"]="help"
)

# 用於存儲解析後的參數
declare -A parsed_args

# 解析參數
parse_args "$@"

declare -A default_flags=(
  ["--appendbashrc"]=true
  ["--disable-convert"]=false
)
APPENDBASHRC=$(parse_flag "appendbashrc")
DISABLE_CONVERT=$(parse_flag "disable_convert")

UBUNTU_CODENAME=${parsed_args["ubuntu_codename"]-focal}
ROSDISTRO=${parsed_args["rosdistro"]-galactic}
WORKSPACE=${parsed_args["workspace"]-ros2_ws}
# APPENDBASHRC=${parsed_args["appendbashrc"]-false}
EXCLUDES=${parsed_args["excludes"]}
IFS=',' read -ra EXCLUDES_ARRAY <<<"$EXCLUDES"

if [ ${#positional_args[@]} -gt 0 ]; then
  # 以第一个位置参数为例
  FIRST_POSITIONAL=${positional_args[0]}
  echo "First Positional Argument: $FIRST_POSITIONAL"
else
  echo "Please provide at least one positional argument."
  exit 1
fi
for arg in "${positional_args[@]}"; do
  echo "  $arg"
done

var1=${positional_args[0]}
var2=${positional_args[1]}
var3=${positional_args[2]}
echo "var1: $var1"
echo "var2: $var2"
echo "var3: $var3"

echo "UBUNTU_CODENAME: $UBUNTU_CODENAME"
echo "ROSDISTRO: $ROSDISTRO"
echo "WORKSPACE: $WORKSPACE"
echo "APPENDBASHRC: $APPENDBASHRC"
echo "DISABLE_CONVERT: $DISABLE_CONVERT"
echo "EXCLUDES: $EXCLUDES"
echo "EXCLUDES_ARRAY: $EXCLUDES_ARRAY"
for ex in "${EXCLUDES_ARRAY[@]}"; do
  echo "$ex"
done

