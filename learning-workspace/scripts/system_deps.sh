#!/usr/bin/env bash
# Sourced by install.sh. Keep platform planning separate from privileged changes.
platform_plan() {
    local distro="$1" like="$2" version="$3"
    SYSTEM_MANAGER=manual
    SERIAL_GROUP=dialout
    SYSTEM_PACKAGES=()
    case " $distro $like " in
        *' ubuntu '*|*' debian '*)
            SYSTEM_MANAGER=apt
            local glib=libglib2.0-0
            if [[ "$distro" == ubuntu && "${version%%.*}" -ge 24 ]] ||
               [[ "$distro" == debian && "${version%%.*}" -ge 13 ]]; then
                glib=libglib2.0-0t64
            fi
            SYSTEM_PACKAGES=(python3 build-essential ca-certificates ffmpeg "$glib" libgl1 libegl1
                libx11-6 libxtst6 libxkbcommon-x11-0 libusb-1.0-0 usbutils v4l-utils can-utils iproute2)
            ;;
        *' fedora '*)
            SYSTEM_MANAGER=dnf
            SYSTEM_PACKAGES=(python3 gcc gcc-c++ make ca-certificates ffmpeg-free glib2 libglvnd-glx
                libglvnd-egl libX11 libXtst libxkbcommon-x11 libusb1 usbutils v4l-utils can-utils iproute)
            ;;
        *' arch '*)
            SYSTEM_MANAGER=pacman
            SERIAL_GROUP=uucp
            SYSTEM_PACKAGES=(python base-devel ca-certificates ffmpeg glib2 libglvnd libx11 libxtst
                libxkbcommon-x11 libusb usbutils v4l-utils can-utils iproute2)
            ;;
    esac
}

check_binary_platform() {
    [[ "$(uname -s)" == Linux && "$(uname -m)" == x86_64 ]] || {
        echo '此版本需要 Linux x86_64；ARM/aarch64 尚未适配。'; return 1;
    }
    local libc version major minor
    libc=$(getconf GNU_LIBC_VERSION 2>/dev/null) || {
        echo '此版本需要 glibc >= 2.28；不支持 Alpine/musl。'; return 1;
    }
    version=${libc#glibc }
    major=${version%%.*}
    minor=${version#*.}; minor=${minor%%.*}
    if ((major < 2 || (major == 2 && minor < 28))); then
        echo "glibc $version 太旧，需要 >= 2.28。"; return 1
    fi
}

install_system_dependencies() {
    local missing=() package group
    command -v sudo >/dev/null || { echo '请先安装 sudo，或由管理员预装系统依赖后使用 --skip-system。'; return 1; }
    case "$SYSTEM_MANAGER" in
        apt)
            # Derivatives may report their own version; use the base system's package catalogue.
            sudo apt-get update
            if apt-cache show libglib2.0-0t64 >/dev/null 2>&1; then
                for package in "${!SYSTEM_PACKAGES[@]}"; do
                    [[ "${SYSTEM_PACKAGES[$package]}" != libglib2.0-0 ]] || SYSTEM_PACKAGES[$package]=libglib2.0-0t64
                done
            fi
            for package in "${SYSTEM_PACKAGES[@]}"; do
                if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -qx 'install ok installed'; then
                    missing+=("$package")
                fi
            done
            ((${#missing[@]} == 0)) || sudo apt-get install -y --no-install-recommends "${missing[@]}"
            ;;
        dnf) sudo dnf install -y "${SYSTEM_PACKAGES[@]}" ;;
        pacman)
            # Refuse a partial system upgrade: refresh/upgrade separately using distro guidance.
            sudo pacman -S --needed --noconfirm "${SYSTEM_PACKAGES[@]}"
            ;;
        *) echo '未适配此系统的包管理器。请预装依赖后使用 --skip-system，参见 docs/SUPPORT.md。'; return 1 ;;
    esac
    local groups=()
    for group in "$SERIAL_GROUP" video; do
        if getent group "$group" >/dev/null && ! id -nG | tr ' ' '\n' | grep -qx "$group"; then
            groups+=("$group")
        fi
    done
    if ((${#groups[@]})); then
        local joined
        joined=$(IFS=,; echo "${groups[*]}")
        sudo usermod -aG "$joined" "$(id -un)"
        echo '已添加串口/摄像头用户组；请注销并重新登录 Linux 后再连接设备。'
    fi
}
