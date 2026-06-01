#!/usr/bin/env python3
"""Remote SSH helper for deploying to Raspberry Pi using paramiko."""
import sys
import paramiko
import os
import getpass

HOST = "10.0.0.205"
USER = "pi"
PASSWORD = "yeahbotros"

def get_client():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASSWORD, timeout=15)
    return client

def run_remote(cmd, check=True, timeout=300):
    """Run a command on the remote host and print output."""
    client = get_client()
    try:
        stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
        out = stdout.read().decode('utf-8', errors='replace')
        err = stderr.read().decode('utf-8', errors='replace')
        exit_code = stdout.channel.recv_exit_status()
        if out:
            print(out, end='')
        if err:
            print(err, end='', file=sys.stderr)
        if check and exit_code != 0:
            print(f"[ERROR] Command failed with exit code {exit_code}: {cmd}", file=sys.stderr)
            sys.exit(exit_code)
        return exit_code, out, err
    finally:
        client.close()

def upload_file(local_path, remote_path):
    """Upload a file via SFTP."""
    client = get_client()
    try:
        sftp = client.open_sftp()
        sftp.put(local_path, remote_path)
        sftp.close()
        print(f"[OK] Uploaded {local_path} -> {remote_path}")
    finally:
        client.close()

def upload_dir(local_dir, remote_dir, exclude=None):
    """Upload a directory recursively via SFTP."""
    if exclude is None:
        exclude = {'.git', 'build', 'install', 'log', '__pycache__', '.claude',
                   'frames_*.gv', 'frames_*.pdf', 'key', '.gitignore'}

    client = get_client()
    try:
        sftp = client.open_sftp()
        _upload_dir_recursive(sftp, local_dir, remote_dir, exclude)
        sftp.close()
    finally:
        client.close()

def _upload_dir_recursive(sftp, local_dir, remote_dir, exclude):
    try:
        sftp.stat(remote_dir)
    except FileNotFoundError:
        sftp.mkdir(remote_dir)

    for item in os.listdir(local_dir):
        if item in exclude:
            continue
        # Skip wildcard patterns
        skip = False
        for pat in exclude:
            if '*' in pat:
                import fnmatch
                if fnmatch.fnmatch(item, pat):
                    skip = True
                    break
        if skip:
            continue

        local_path = os.path.join(local_dir, item)
        remote_path = f"{remote_dir}/{item}"

        if os.path.islink(local_path):
            # Skip symlinks - they point to third-party stuff
            continue
        elif os.path.isdir(local_path):
            _upload_dir_recursive(sftp, local_path, remote_path, exclude)
        elif os.path.isfile(local_path):
            try:
                sftp.put(local_path, remote_path)
                print(f"  {remote_path}")
            except Exception as e:
                print(f"  [SKIP] {remote_path}: {e}", file=sys.stderr)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: deploy_remote.py <command> [args...]")
        print("  run <cmd>         - Run remote command")
        print("  upload <src> <dst> - Upload file")
        print("  check             - Check remote environment")
        sys.exit(1)

    action = sys.argv[1]
    if action == 'run':
        cmd = ' '.join(sys.argv[2:])
        run_remote(cmd)
    elif action == 'upload':
        upload_file(sys.argv[2], sys.argv[3])
    elif action == 'check':
        run_remote("uname -a && echo '---' && cat /etc/os-release | head -3 && echo '---' && ls /opt/ros/ 2>/dev/null && echo '---' && python3 --version && echo '---' && df -h /home/pi && echo '---' && free -h | head -2 && echo '---' && which colcon 2>/dev/null; echo '---' && cat ~/.bashrc 2>/dev/null | tail -20")
