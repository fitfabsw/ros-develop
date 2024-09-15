#!/bin/bash
WORKSPACE=$1
VCS_REPOS=$2
vcs_source="$VCS_REPOS"
WORKSPACEPATH="$HOME/$WORKSPACE"

if [ -d $HOME/"$WORKSPACE" ]; then
	if [ -f "$vcs_source" ]; then
		echo "vcs_source: $vcs_source exists"
	else
		echo "ERROR: vcs source $vcs_source does not exist"
		exit 1
	fi
	echo "Copy vcs_source $vcs_source to workspace $WORKSPACEPATH"
	TARGET_PATH="$WORKSPACEPATH/$(basename "$VCS_REPOS")"
	# echo "TARGET_PATH: $TARGET_PATH"
	if cmp -s "$vcs_source" $TARGET_PATH; then
		echo "vcs_source $vcs_source is the same as $TARGET_PATH"
		exit 0
	else
		echo "cp -f $vcs_source $HOME/$WORKSPACE"
		cp -f "$vcs_source" $HOME/"$WORKSPACE"
	fi
else
	echo "ERROR: workspace $WORKSPACE does not exist"
	exit 1
fi
