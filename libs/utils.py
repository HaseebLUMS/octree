import os
import sys

directory_path = sys.argv[1]
prefix = sys.argv[2]
suffix = sys.argv[3]

items = os.listdir(directory_path)

# Filter out only the directories from the list
directories = [f"\"{prefix}{item}{suffix}\"" for item in items if os.path.isdir(os.path.join(directory_path, item))]

# Print the list of directory names
res = ""
for directory in directories:
    res += directory + ",\n"

print(res)
