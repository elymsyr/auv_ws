import os

def print_tree(startpath, prefix=''):
    """
    Prints a directory tree structure.
    """
    # Exclude common Python and build artifacts for a cleaner view
    exclude_dirs = {'__pycache__', '.git', 'build', 'install', 'log', 'libtorch'}
    
    # Get directory contents and sort them
    try:
        dir_contents = sorted(os.listdir(startpath))
    except OSError as e:
        print(f"Error accessing {startpath}: {e}")
        return

    # Filter out excluded directories
    filtered_contents = [item for item in dir_contents if item not in exclude_dirs]
    
    pointers = ['├── '] * (len(filtered_contents) - 1) + ['└── ']

    for pointer, path in zip(pointers, filtered_contents):
        full_path = os.path.join(startpath, path)
        print(prefix + pointer + path)
        
        if os.path.isdir(full_path):
            # For directories, extend the prefix accordingly before recursing
            extension = '│   ' if pointer == '├── ' else '    '
            print_tree(full_path, prefix + extension)

if __name__ == '__main__':
    # Get the directory where the script is located, or use the current working directory
    # Using '.' for current working directory is simplest.
    start_dir = '.'
    
    print(f"{start_dir}/")
    print_tree(start_dir)