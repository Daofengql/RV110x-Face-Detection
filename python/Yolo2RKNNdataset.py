import os

image_folder = "/home/daofeng/shares/sharefile/facevalid"
output_file = "retinaface_dataset.txt"
valid_extensions = [".jpg", ".jpeg", ".png"]

image_paths = []
for root, dirs, files in os.walk(image_folder):
 for file in files:
     if os.path.splitext(file)[1].lower() in valid_extensions:
         image_paths.append(os.path.join(root, file))

with open(output_file, "w") as f:
 for path in image_paths:
     f.write(path + "\n")

print(f"路径文件已生成：{output_file}")

