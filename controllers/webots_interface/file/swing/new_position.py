# 打开文件进行读取
with open('SonnieControl/upbody/ArmControl/file/swing/joint_left2.txt', 'r') as file:
    lines = file.readlines()

# 打开文件以写入修改后的内容
with open('SonnieControl/upbody/ArmControl/file/swing/joint_left2_whole.txt', 'w') as file:
    for line in lines:
        # 剔除行尾的换行符
        line = line.strip()
        # 在行尾添加0.0,0.0,0.0，并写入文件
        modified_line = f'{line},0.0,0.0,0.0\n'
        file.write(modified_line)

# 打开文件进行读取
with open('SonnieControl/upbody/ArmControl/file/swing/joint_right2.txt', 'r') as file:
    lines = file.readlines()

# 打开文件以写入修改后的内容
with open('SonnieControl/upbody/ArmControl/file/swing/joint_right2_whole.txt', 'w') as file:
    for line in lines:
        # 剔除行尾的换行符
        line = line.strip()
        # 在行尾添加0.0,0.0,0.0，并写入文件
        modified_line = f'{line},0.0,0.0,0.0\n'
        file.write(modified_line)


print("文件已成功修改并保存为'joint_right2_whole.txt'")
