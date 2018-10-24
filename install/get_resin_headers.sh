#!/bin/bash

files_url='https://files.resin.io' # URL exporting S3 XML

# Output arguments to stderr.
function err()
{
	echo "$@">&2
}

# Output arguments to stderr and halt with non-zero exit code.
function fatal()
{
	err "$@"
	exit 1
}

# Output usage and halt.
function usage()
{
	err   "usage: $0 <device type> <version> <module path>"
	fatal "   or: $0 --list - list available devices and versions"
}

function push()
{
	pushd $1 >/dev/null
}

function pop()
{
	popd >/dev/null
}

# Super-basic XML parsing.
# See http://stackoverflow.com/a/2608159
function rdom()
{
	local IFS=\>
	read -d \< key val
}

# Retrieves all available kernel header archives.
# args: $1 - marker
#       $2 - if non-empty use list mode
#       $3 - device search pattern (default .*)
#       $4 - version search pattern (default .*)
function get_header_paths()
{
	local marker="$1"
	local list_mode="$2"
	local dev_pat="${3:-.*}"
	local ver_pat="${4:-.*}"

	local pattern="^images/($dev_pat)/($ver_pat)/kernel_modules_headers"

	local last_marker=''

	while rdom; do
		local path="$val"

		if [[ "$key" = 'Key' ]]; then
			last_marker=$val

			[[ "$val" =~ $(echo "$pattern" | sed -e 's/+/\\+/g') ]] || continue

			local device="${BASH_REMATCH[1]}"
			local version="${BASH_REMATCH[2]}"

			if [[ -n "$list_mode" ]]; then
				echo $device $version
			else
				echo $path
			fi
		fi
	done <<<$(curl --silent "$files_url?marker=$marker")

	# If we have seen all of the available data then the last marker we've
	# seen will be empty, otherwise we need to recurse to retrieve the rest
	# of the data.
	[[ -z "$last_marker" ]] || get_header_paths "$last_marker" "$list_mode" "$dev_pat" "$ver_pat"
}

# List available devices and versions.
function list_versions()
{
	get_header_paths '' 'y' | while read device version path; do
		printf "%-30s %-30s\n" $device $version
	done
}

# Retrieve kernel module headers from the specified remote path
function download_headers()
{
	local path="$1"
	local pattern="^images/(.*)/(.*)/"
	[[ "$path" =~ $pattern ]] || fatal "Invalid path '$path'?!"

	local device="${BASH_REMATCH[1]}"
	local version="${BASH_REMATCH[2]}"
	local output_dir="${module_dir}_${device}_${version}"

	filename=$(basename $path)
	url="$files_url/$path"

	if ! wget $(echo "$url" | sed -e 's/+/%2B/g') -q; then
		err "ERROR: $path: Could not retrieve $url, skipping."
		return
	fi

	if ! tar -xf $filename; then
		err "ERROR: $path: Unable to extract $filename, skipping."
		return
	fi

	rm $filename
}

if [[ "$1" = "--list" ]]; then
	list_versions
	exit
elif [[ $# -lt 2 ]]; then
	usage
fi

device="$1"
version="$2"

seen=''
for path in $(get_header_paths '' '' "$device" "$version"); do
	echo "Building $path..."

	download_headers $path
	seen='y'
done

if [[ -z "$seen" ]]; then
  fatal "Could not find headers for '$device' at version '$version', running $0 --list"
  list_versions
fi
