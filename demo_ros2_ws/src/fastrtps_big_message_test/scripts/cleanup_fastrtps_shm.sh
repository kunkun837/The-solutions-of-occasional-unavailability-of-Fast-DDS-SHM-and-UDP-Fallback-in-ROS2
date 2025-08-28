#!/bin/bash      
# 使用方法：
# 1./cleanup_fastrtps_shm.sh          # 安全清理（检查进程）
# 2./cleanup_fastrtps_shm.sh force    # 强制清理所有

set -e           #设置脚本在​​任何命令失败时立即退出​​（防止错误累积）
#定义终端颜色代码
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}
print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}
print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}
print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否有ROS2进程在运行
check_active_ros2_processes() {
    local count=$(pgrep -f 'ros2' 2>/dev/null | wc -l)
    echo $count
}
#显示SHM文件和信号量
show_fastrtps_files() {
    #获取共享内存段文件
    local shm_files=$(ls -1 /dev/shm/fastrtps_* 2>/dev/null | head -20)
    #获取信号量文件
    local sem_files=$(ls -1 /dev/shm/sem.fastrtps_* 2>/dev/null | head -10)
    local has_files=false                                                   #初始化文件存在标志
    if [ -n "$shm_files" ] || [ -n "$sem_files" ]; then
        has_files=true
        echo -e "${YELLOW}FastRTPS SHM files and semaphores:${NC}"
        # 显示共享内存段文件
        if [ -n "$shm_files" ]; then
            echo "$shm_files" | while read file; do
                local size=$(stat -c%s "$file" 2>/dev/null || echo "0")
                local time=$(stat -c%y "$file" 2>/dev/null || echo "unknown")
                echo "  - [SHM] $(basename $file) (${size} bytes, $time)"
            done
        fi
        # 显示信号量文件
        if [ -n "$sem_files" ]; then
            echo "$sem_files" | while read file; do
                local size=$(stat -c%s "$file" 2>/dev/null || echo "0")
                local time=$(stat -c%y "$file" 2>/dev/null || echo "unknown")
                echo "  - [SEM] $(basename $file) (${size} bytes, $time)"
            done
        fi
        # 统计总数
        local shm_total=$(ls -1 /dev/shm/fastrtps_* 2>/dev/null | wc -l)
        local sem_total=$(ls -1 /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
        local total_shown=$(($(echo "$shm_files" | wc -l) + $(echo "$sem_files" | wc -l)))
        local total_actual=$((shm_total + sem_total))
        
        if [ $total_actual -gt $total_shown ]; then
            echo "  ... and $((total_actual - total_shown)) more files"
        fi
        echo "  Total: $shm_total SHM files, $sem_total semaphore files"
    else
        echo -e "${GREEN}No FastRTPS SHM files or semaphores found${Colors.NC}"
    fi
}
#安全清理模式,在没有活跃进程时清理残留的共享内存段及其_el独占锁文件；
safe_cleanup() {
    print_info "Starting safe cleanup..."
    local active_processes=$(check_active_ros2_processes)
    if [ $active_processes -gt 0 ]; then
        print_warning "Found $active_processes active ROS2 processes"
        print_warning "Safe cleanup skipped - will not clean files while ROS2 processes are running"
        return 0
    else
        print_info "No active ROS2 processes found - proceeding with cleanup"
    fi
    local cleaned_count=0
    # 清理共享内存段文件,只清理16位UUID格式的Discovery文件
    for file in /dev/shm/fastrtps_*; do
        [ -f "$file" ] || continue
        local filename=$(basename "$file")
        local should_clean=false
        # 匹配 fastrtps_<16位小写hex> 或 fastrtps_<16位小写hex>_el
        if [[ "$filename" =~ ^fastrtps_[0-9a-f]{16}(_el)?$ ]]; then
            should_clean=true
            print_info "Cleaning Discovery SHM file: $filename"
        fi
        if [ "$should_clean" = true ]; then
            if rm -f "$file" 2>/dev/null; then
                cleaned_count=$((cleaned_count + 1))
                print_success "  Removed: $filename"
            else
                print_error "  Failed to remove: $filename"
            fi
        fi
    done
    print_success "Safe cleanup completed: $cleaned_count files removed"
}
#强制清理模式清理/dev/shm下的所有共享内存相关文件（共享内存段文件、端口文件以及锁文件）
force_cleanup() {
    print_warning "Starting FORCE cleanup (removes ALL FastRTPS files and semaphores)..."
    
    local active_processes=$(check_active_ros2_processes)
    if [ $active_processes -gt 0 ]; then
        print_error "WARNING: Found $active_processes active ROS2 processes!"
        print_error "Force cleanup may interfere with running nodes!"
        echo -n "Continue? (y/N): "
        read -r response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            print_info "Cleanup cancelled"
            return 0
        fi
    fi
    # 统计要删除的文件
    local shm_files=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
    local sem_files=$(ls /dev/shm/sem.fastrtps_* 2>/dev/null | wc -l)
    local total_files=$((shm_files + sem_files))
    if [ $total_files -eq 0 ]; then
        print_info "No FastRTPS files to clean"
        return 0
    fi
    print_info "Removing $shm_files SHM files and $sem_files semaphore files (total: $total_files)..."
    local success_count=0
    local error_count=0
    # 删除共享内存段文件
    if [ $shm_files -gt 0 ]; then
        if rm -f /dev/shm/fastrtps_* 2>/dev/null; then
            success_count=$((success_count + shm_files))
            print_success "Removed $shm_files SHM files"
        else
            error_count=$((error_count + shm_files))
            print_error "Failed to remove some SHM files"
        fi
    fi
    # 删除信号量文件
    if [ $sem_files -gt 0 ]; then
        if rm -f /dev/shm/sem.fastrtps_* 2>/dev/null; then
            success_count=$((success_count + sem_files))
            print_success "Removed $sem_files semaphore files"
        else
            error_count=$((error_count + sem_files))
            print_error "Failed to remove some semaphore files"
        fi
    fi
    if [ $error_count -eq 0 ]; then
        print_success "Force cleanup completed: $success_count files removed"
    else
        print_error "Some files could not be removed (permission issues?)"
        print_info "Successfully removed: $success_count files, Failed: $error_count files"
    fi
}
# 主函数
main() {
    echo "FastRTPS SHM Cleanup Tool"
    echo "========================="
    # 显示当前状态
    show_fastrtps_files
    echo ""
    case "${1:-safe}" in
        safe)
            safe_cleanup
            ;;
        force)
            force_cleanup
            ;;
        show)
            # 只显示，不清理
            ;;
        help|--help|-h)
            echo "Usage: $0 [safe|force|show|help]"
            echo ""
            echo "  safe  (default) - Safe cleanup, only when no ROS2 processes are running"
            echo "                   Cleans Discovery files: fastrtps_<16-hex-uuid>[_el]"
            echo "                   Cleans semaphores: sem.fastrtps_<16-hex-uuid>[_el]"
            echo "  force           - Force cleanup ALL FastRTPS files and semaphores"
            echo "  show            - Just show current files"
            echo "  help            - Show this help"
            echo ""
            echo "Examples:"
            echo "  $0              # Safe cleanup"
            echo "  $0 force        # Force cleanup"
            echo "  $0 show         # Just show files"
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use '$0 help' for usage information"
            exit 1
            ;;
    esac
    echo ""
    echo "After cleanup:"
    show_fastrtps_files
}
# 检查权限
if [ ! -w /dev/shm ]; then
    print_error "/dev/shm is not writable!"
    print_info "Try running with sudo or check permissions"
    exit 1
fi
# 运行主函数
main "$@"
