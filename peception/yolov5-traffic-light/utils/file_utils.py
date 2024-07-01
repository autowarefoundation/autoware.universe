# -*- coding: utf-8 -*-
"""
# --------------------------------------------------------
# @Project: torch-Face-Recognize-Pipeline
# @Author : panjq
# @E-mail : pan_jinquan@163.com
# @Date   : 2019-12-31 09:11:25
# --------------------------------------------------------
"""
import glob
import os
import time
import os, shutil
import numpy as np
import json
import random
import os
import subprocess
import concurrent.futures
import numbers
from datetime import datetime


def get_time(format="S"):
    """
    :param format:
    :return:
    """
    if format in ["S", "s"]:
        # time = datetime.strftime(datetime.now(), '%Y%m%d_%H%M%S')
        time = datetime.strftime(datetime.now(), '%Y%m%d%H%M%S')
    elif format in ["P", "p"]:
        # 20200508_143059_379116
        time = datetime.strftime(datetime.now(), '%Y%m%d_%H%M%S_%f')
        time = time[:-2]
    else:
        time = (str(datetime.now())[:-10]).replace(' ', '-').replace(':', '-')
    return time


def get_kwargs_name(**kwargs):
    prefix = []
    for k, v in kwargs.items():
        if isinstance(v, list):
            v = [str(l) for l in v]
            prefix += v
        else:
            f = "{}_{}".format(k, v)
            prefix.append(f)
    prefix = "_".join(prefix)
    return prefix


def combine_flags(flags: list, use_time=True, info=True):
    """
    :param flags:
    :param info:
    :return:
    """
    out_flags = []
    for f in flags:
        if isinstance(f, dict):
            f = get_kwargs_name(**f)
        out_flags.append(f)
    if use_time:
        out_flags += [get_time()]
    out_flags = [str(f) for f in out_flags if f]
    out_flags = "_".join(out_flags)
    if info:
        print(out_flags)
    return out_flags


class WriterTXT(object):
    """ write data in txt files"""

    def __init__(self, filename, mode='w'):
        self.f = None
        if filename:
            self.f = open(filename, mode=mode)

    def write_line_str(self, line_str, endline="\n"):
        if self.f:
            line_str = line_str + endline
            self.f.write(line_str)
            self.f.flush()

    def write_line_list(self, line_list, endline="\n"):
        if self.f:
            for line_list in line_list:
                # 将list转为string
                line_str = " ".join('%s' % id for id in line_list)
                self.write_line_str(line_str, endline=endline)
            self.f.flush()

    def close(self):
        if self.f:
            self.f.close()


def parser_classes(class_name):
    """
    :return:
    """
    if isinstance(class_name, str):
        class_name = read_data(class_name, split=None)
    elif isinstance(class_name, numbers.Number):
        class_name = [i for i in range(int(class_name))]
    if isinstance(class_name, list):
        class_dict = {class_name: i for i, class_name in enumerate(class_name)}
    elif isinstance(class_name, dict):
        class_dict = class_name
    else:
        class_dict = None
    return class_name, class_dict


def read_json_data(json_path):
    """
    读取数据
    :param json_path:
    :return:
    """
    with open(json_path, 'r') as f:
        json_data = json.load(f)
    return json_data


def write_json_path(out_json_path, json_data):
    """
    写入 JSON 数据
    :param out_json_path:
    :param json_data:
    :return:
    """
    with open(out_json_path, 'w', encoding="utf-8") as f:
        json.dump(json_data, f, indent=4, ensure_ascii=False)


def write_data(filename, content_list, split=" ", mode='w'):
    """保存list[list[]]的数据到txt文件
    :param filename:文件名
    :param content_list:需要保存的数据,type->list
    :param mode:读写模式:'w' or 'a'
    :return: void
    """
    with open(filename, mode=mode, encoding='utf-8') as f:
        for line_list in content_list:
            # 将list转为string
            line = "{}".format(split).join('%s' % id for id in line_list)
            f.write(line + "\n")
        f.flush()


def write_list_data(filename, list_data, mode='w'):
    """保存list[]的数据到txt文件，每个元素分行
    :param filename:文件名
    :param list_data:需要保存的数据,type->list
    :param mode:读写模式:'w' or 'a'
    :return: void
    """
    with open(filename, mode=mode, encoding='utf-8') as f:
        for line in list_data:
            # 将list转为string
            f.write(str(line) + "\n")
        f.flush()


def read_data(filename, split=" ", convertNum=True):
    """
    读取txt数据函数
    :param filename:文件名
    :param split   :分割符
    :param convertNum :是否将list中的string转为int/float类型的数字
    :return: txt的数据列表
    Python中有三个去除头尾字符、空白符的函数，它们依次为:
    strip： 用来去除头尾字符、空白符(包括\n、\r、\t、' '，即：换行、回车、制表符、空格)
    lstrip：用来去除开头字符、空白符(包括\n、\r、\t、' '，即：换行、回车、制表符、空格)
    rstrip：用来去除结尾字符、空白符(包括\n、\r、\t、' '，即：换行、回车、制表符、空格)
    注意：这些函数都只会删除头和尾的字符，中间的不会删除。
    """
    with open(filename, mode="r", encoding='utf-8') as f:
        content_list = f.readlines()
        if split is None:
            content_list = [content.rstrip() for content in content_list]
            return content_list
        else:
            content_list = [content.rstrip().split(split) for content in content_list]
        if convertNum:
            for i, line in enumerate(content_list):
                line_data = []
                for l in line:
                    if is_int(l):  # isdigit() 方法检测字符串是否只由数字组成,只能判断整数
                        line_data.append(int(l))
                    elif is_float(l):  # 判断是否为小数
                        line_data.append(float(l))
                    else:
                        line_data.append(l)
                content_list[i] = line_data
    return content_list


def read_line_image_label(line_image_label):
    '''
    line_image_label:[image_id,boxes_nums,x1, y1, w, h, label_id,x1, y1, w, h, label_id,...]
    :param line_image_label:
    :return:
    '''
    line_image_label = line_image_label.strip().split()
    image_id = line_image_label[0]
    boxes_nums = int(line_image_label[1])
    box = []
    label = []
    for i in range(boxes_nums):
        x = float(line_image_label[2 + 5 * i])
        y = float(line_image_label[3 + 5 * i])
        w = float(line_image_label[4 + 5 * i])
        h = float(line_image_label[5 + 5 * i])
        c = int(line_image_label[6 + 5 * i])
        if w <= 0 or h <= 0:
            continue
        box.append([x, y, x + w, y + h])
        label.append(c)
    return image_id, box, label


def read_lines_image_labels(filename):
    """
    :param filename:
    :return:
    """
    boxes_label_lists = []
    with open(filename) as f:
        lines = f.readlines()
        for line in lines:
            image_id, box, label = read_line_image_label(line)
            boxes_label_lists.append([image_id, box, label])
    return boxes_label_lists


def save_file_root_list(file_root: str,
                        out_path=None,
                        postfix=["*.jpg"],
                        replace_postfix=False,
                        add_dirname=False,
                        shuffle=False):
    """
    保存file_dir目录下所有后缀名为postfix的文件
    :param file_root: 文件根目录
    :param out_path: 保存列表的路径,默认为file_dir的上一级目录"file_id.txt"
    :param postfix: 文件后缀名,支持多个后缀格式
    :param remove_postfix: <bool> or <str>文件列表是否包含后缀名
    :param dirname: 文件列表是否增加父目录名称
    :return:
    """
    file_dir_len = len(file_root)
    if not file_root.endswith(os.sep):
        file_dir_len = file_dir_len + 1
    dirname = os.path.basename(file_root)
    anno_list = get_files(file_root, postfix=postfix)
    if shuffle:
        random.seed(100)
        random.shuffle(anno_list)
    image_idx = []
    for path in anno_list:
        # basename = os.path.basename(path)
        basename = path[file_dir_len:]
        if replace_postfix:
            basename = basename.split(".")[0] + replace_postfix
        elif replace_postfix == "":
            basename = basename.split(".")[0]
        if add_dirname:
            basename = os.path.join(dirname, basename)
        image_idx.append(basename)
    if not out_path:
        out_path = os.path.join(os.path.dirname(file_root), "file_id.txt")
    print("num files:{},out_path:{}".format(len(image_idx), out_path))
    write_list_data(out_path, image_idx)
    return image_idx


def is_int(str):
    """
    判断是否为整数
    :param str:
    :return:
    """
    try:
        x = int(str)
        return isinstance(x, int)
    except ValueError:
        return False


def is_float(str):
    """
    判断是否为整数和小数
    :param str:
    :return:
    """
    try:
        x = float(str)
        return isinstance(x, float)
    except ValueError:
        return False


def list2str(content_list):
    """
    convert list to string
    :param content_list:
    :return:
    """
    content_str_list = []
    for line_list in content_list:
        line_str = " ".join('%s' % id for id in line_list)
        content_str_list.append(line_str)
    return content_str_list


def get_images_list(image_dir, postfix=['*.jpg'], basename=False):
    '''
    获得文件列表
    :param image_dir: 图片文件目录
    :param postfix: 后缀名，可是多个如，['*.jpg','*.png']
    :param basename: 返回的列表是文件名（True），还是文件的完整路径(False)
    :return:
    '''
    images_list = []
    for format in postfix:
        image_format = os.path.join(image_dir, format)
        image_list = glob.glob(image_format)
        if not image_list == []:
            images_list += image_list
    images_list = sorted(images_list)
    if basename:
        images_list = get_basename(images_list)
    return images_list


def get_basename(file_list):
    """
    get files basename
    :param file_list:
    :return:
    """
    dest_list = []
    for file_path in file_list:
        basename = os.path.basename(file_path)
        dest_list.append(basename)
    return dest_list


def randam_select_images(image_list, nums, shuffle=True):
    """
    randam select nums images
    :param image_list:
    :param nums:
    :param shuffle:
    :return:
    """
    image_nums = len(image_list)
    if image_nums <= nums:
        return image_list
    if shuffle:
        random.seed(100)
        random.shuffle(image_list)
    out = image_list[:nums]
    return out


def remove_dir(dir):
    """
    remove directory
    :param dir:
    :return:
    """
    if os.path.exists(dir):
        shutil.rmtree(dir)


def get_prefix_files(file_dir, prefix):
    """
    :param file_dir:
    :param prefix: "best*"
    :return:
    """
    file_list = glob.glob(os.path.join(file_dir, prefix))
    return file_list


def remove_prefix_files(file_dir, prefix):
    """
    :param file_dir:
    :param prefix: "best*"
    :return:
    """
    file_list = get_prefix_files(file_dir, prefix)
    for file in file_list:
        if os.path.isfile(file):
            remove_file(file)
        elif os.path.isdir(file):
            remove_dir(file)


def remove_file(path):
    """
    remove files
    :param path:
    :return:
    """
    if os.path.exists(path):
        os.remove(path)


def remove_file_list(file_list):
    """
    remove file list
    :param file_list:
    :return:
    """
    for file_path in file_list:
        remove_file(file_path)


def copy_dir_multi_thread(sync_source_root, sync_dest_dir, dataset, max_workers=1):
    """
    :param sync_source_dir:
    :param sync_dest_dir:
    :param dataset:
    :return:
    """

    def rsync_cmd(source_dir, dest_dir):
        cmd_line = "rsync -a {0} {1}".format(source_dir, dest_dir)
        # subprocess.call(cmd_line.split())
        subprocess.call(cmd_line)

    sync_dest_dir = sync_dest_dir.rstrip('/')

    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as executor:
        future_to_rsync = {}
        for source_dir in dataset:
            sync_source_dir = os.path.join(sync_source_root, source_dir.strip('/'))
            future_to_rsync[executor.submit(rsync_cmd, sync_source_dir, sync_dest_dir)] = source_dir

        for future in concurrent.futures.as_completed(future_to_rsync):
            source_dir = future_to_rsync[future]
            try:
                _ = future.result()
            except Exception as exc:
                print("%s copy data generated an exception: %s" % (source_dir, exc))
            else:
                print("%s copy data successful." % (source_dir,))


def copy_dir_delete(src, dst):
    """
    copy src directory to dst directory,will detete the dst same directory
    :param src:
    :param dst:
    :return:
    """
    if os.path.exists(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    # time.sleep(3 / 1000.)


def copy_dir(src, dst):
    """ copy src-directory to dst-directory, will cover the same files"""
    if not os.path.exists(src):
        print("\nno src path:{}".format(src))
        return
    for root, dirs, files in os.walk(src, topdown=False):
        dest_path = os.path.join(dst, os.path.relpath(root, src))
        if not os.path.exists(dest_path):
            os.makedirs(dest_path)
        for filename in files:
            copy_file(
                os.path.join(root, filename),
                os.path.join(dest_path, filename)
            )


def move_file(srcfile, dstfile):
    """ 移动文件或重命名"""
    if not os.path.isfile(srcfile):
        print("%s not exist!" % (srcfile))
    else:
        fpath, fname = os.path.split(dstfile)  # 分离文件名和路径
        if not os.path.exists(fpath):
            os.makedirs(fpath)  # 创建路径
        shutil.move(srcfile, dstfile)
        # print("copy %s -> %s"%( srcfile,dstfile))
        # time.sleep(1 / 1000.)


def copy_file(srcfile, dstfile):
    """
    copy src file to dst file
    :param srcfile:
    :param dstfile:
    :return:
    """
    if not os.path.isfile(srcfile):
        print("%s not exist!" % (srcfile))
    else:
        fpath, fname = os.path.split(dstfile)  # 分离文件名和路径
        if not os.path.exists(fpath):
            os.makedirs(fpath)  # 创建路径
        shutil.copyfile(srcfile, dstfile)  # 复制文件
        # print("copy %s -> %s"%( srcfile,dstfile))
        # time.sleep(1 / 1000.)


def copy_file_to_dir(srcfile, des_dir):
    if not os.path.isfile(srcfile):
        print("%s not exist!" % (srcfile))
    else:
        fpath, fname = os.path.split(srcfile)  # 分离文件名和路径
        if not os.path.exists(des_dir):
            os.makedirs(des_dir)  # 创建路径
        dstfile = os.path.join(des_dir, fname)
        shutil.copyfile(srcfile, dstfile)  # 复制文件


def move_file_to_dir(srcfile, des_dir):
    if not os.path.isfile(srcfile):
        print("%s not exist!" % (srcfile))
    else:
        fpath, fname = os.path.split(srcfile)  # 分离文件名和路径
        if not os.path.exists(des_dir):
            os.makedirs(des_dir)  # 创建路径
        dstfile = os.path.join(des_dir, fname)
        # shutil.copyfile(srcfile, dstfile)  # 复制文件
        move_file(srcfile, dstfile)  # 复制文件


def merge_dir(src, dst, sub, merge_same):
    src_dir = os.path.join(src, sub)
    dst_dir = os.path.join(dst, sub)

    if not os.path.exists(src_dir):
        print("\nno src path:{}".format(src))
        return
    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)
    elif not merge_same:
        t = get_time()
        dst_dir = os.path.join(dst, sub + "_{}".format(t))
        print("have save sub:{}".format(dst_dir))
    copy_dir(src_dir, dst_dir)


# def merge_dir(src, dst, merge_same=False):
#     '''
#     move and merge files, move/merge files from source directory to dst directory
#     root 所指的是当前正在遍历的这个文件夹的本身的地址
#     dirs 是一个 list ，内容是该文件夹中所有的目录的名字(不包括子目录)
#     files 同样是 list , 内容是该文件夹中所有的文件(不包括子目录)
#     :param source_dir:
#     :param dest_dir:
#     :return:
#     '''
#     if not os.path.exists(src):
#         print("\nno src path:{}".format(src))
#     for root, dirs, files in os.walk(src, topdown=False):
#         dest_path = os.path.join(dst, os.path.relpath(root, src))
#         if not os.path.exists(dest_path):
#             os.makedirs(dest_path)
#         for filename in files:
#             copy_file(
#                 os.path.join(root, filename),
#                 os.path.join(dest_path, filename)
#             )


def create_dir(parent_dir, dir1=None, filename=None):
    """
    create directory
    :param parent_dir:
    :param dir1:
    :param filename:
    :return:
    """
    out_path = parent_dir
    if dir1:
        out_path = os.path.join(parent_dir, dir1)
    if not os.path.exists(out_path):
        os.makedirs(out_path)
    if filename:
        out_path = os.path.join(out_path, filename)
    return out_path


def create_file_path(filename):
    """
    create file in path
    :param filename:
    :return:
    """
    basename = os.path.basename(filename)
    dirname = os.path.dirname(filename)
    out_path = create_dir(dirname, dir1=None, filename=basename)
    return out_path


def merge_list(data1, data2):
    '''
    将两个list进行合并
    :param data1:
    :param data2:
    :return:返回合并后的list
    '''
    if not len(data1) == len(data2):
        return
    all_data = []
    for d1, d2 in zip(data1, data2):
        if not isinstance(d1, list):
            d1 = [d1]
        if not isinstance(d2, list):
            d2 = [d2]
        all_data.append(d1 + d2)
    return all_data


def split_list(data, split_index=1):
    '''
    将data切分成两部分
    :param data: list
    :param split_index: 切分的位置
    :return:
    '''
    data1 = []
    data2 = []
    for d in data:
        d1 = d[0:split_index]
        d2 = d[split_index:]
        data1.append(d1)
        data2.append(d2)
    return data1, data2


def getFilePathList(file_dir):
    '''
    获取file_dir目录下，所有文本路径，包括子目录文件
    :param rootDir:
    :return:
    '''
    filePath_list = []
    for walk in os.walk(file_dir):
        part_filePath_list = [os.path.join(walk[0], file).replace("\\", "/") for file in walk[2]]
        filePath_list.extend(part_filePath_list)
    return filePath_list


def get_sub_directory_list(input_dir):
    '''
    当前路径下所有子目录
    :param input_dir:
    :return:
    '''
    dirs_list = []
    for root, dirs, files in os.walk(input_dir):
        dirs_list = dirs
        break
    # print(root)   # 当前目录路径
    # print(dirs)   # 当前路径下所有子目录
    # print(files)  # 当前路径下所有非目录子文件
    dirs_list.sort()
    return dirs_list


def get_sub_paths(path_list=[], parent=""):
    """
    :param path_list:
    :param parent:
    :return:
    """
    out_dir_list = []
    if not parent.endswith(os.sep):
        parent = parent + os.sep
    l = len(parent)
    for path in path_list:
        if parent in path:
            p = path[l:]
        out_dir_list.append(p)
    return out_dir_list


def get_files_lists(image_dir, subname="", postfix=["*.jpg", "*.png"], shuffle=False):
    """
    读取文件和列表: list,*.txt ,image path, directory
    :param image_dir: list,*.txt ,image path, directory
    :param subname: "JPEGImages"
    :return:
    """
    if isinstance(image_dir, list):
        image_list = image_dir
    elif image_dir.endswith(".txt"):
        data_root = os.path.dirname(image_dir)
        image_list = read_data(image_dir)
        image_list = [os.path.join(data_root, subname, str(name[0]) + ".jpg") for name in image_list]
    elif os.path.isdir(image_dir):
        image_list = get_files(image_dir, postfix=postfix)
    elif os.path.isfile(image_dir):
        image_list = [image_dir]
    else:
        raise Exception("Error:{}".format(image_dir))

    if shuffle:
        random.seed(100)
        random.shuffle(image_list)
    return image_list


def get_files(file_dir, postfix=None):
    '''
    获得file_dir目录下，后缀名为postfix所有文件列表，包括子目录
    :param file_dir:
    :param postfix: ['*.jpg','*.png'],postfix=None表示全部文件
    :return:
    '''
    file_list = []
    filePath_list = getFilePathList(file_dir)
    if postfix is None:
        file_list = filePath_list
    else:
        postfix = [p.split('.')[-1] for p in postfix]
        for file in filePath_list:
            basename = os.path.basename(file)  # 获得路径下的文件名
            postfix_name = basename.split('.')[-1]
            if postfix_name.lower() in postfix:
                file_list.append(file)
    file_list.sort()
    return file_list


def get_files_labels(files_dir, postfix=None):
    '''
    获取files_dir路径下所有文件路径，以及labels,其中labels用子级文件名表示
    files_dir目录下，同一类别的文件放一个文件夹，其labels即为文件的名
    :param files_dir:
    :postfix 后缀名
    :return:filePath_list所有文件的路径,label_list对应的labels
    '''
    # filePath_list = getFilePathList(files_dir)
    filePath_list = get_files(files_dir, postfix=postfix)
    print("files nums:{}".format(len(filePath_list)))
    # 获取所有样本标签
    label_list = []
    for filePath in filePath_list:
        label = filePath.split(os.sep)[-2]
        label_list.append(label)

    labels_set = list(set(label_list))
    # print("labels:{}".format(labels_set))

    # 标签统计计数
    # print(pd.value_counts(label_list))
    return filePath_list, label_list


def decode_label(label_list, name_table):
    '''
    根据name_table解码label
    :param label_list:
    :param name_table:
    :return:
    '''
    name_list = []
    for label in label_list:
        name = name_table[label]
        name_list.append(name)
    return name_list


def encode_label(name_list, name_table, unknow=0):
    '''
    根据name_table，编码label
    :param name_list:
    :param name_table:
    :param unknow :未知的名称，默认label为0,一般在name_table中index=0是背景，未知的label也当做背景处理
    :return:
    '''
    label_list = []
    # name_table = {name_table[i]: i for i in range(len(name_table))}
    for name in name_list:
        if name in name_table:
            index = name_table.index(name)
        else:
            index = unknow
        label_list.append(index)
    return label_list


def list2dict(data):
    """
    convert list to dict
    :param data:
    :return:
    """
    data = {data[i]: i for i in range(len(data))}
    return data


def print_dict(dict_data, save_path):
    """
    print dict info
    :param dict_data:
    :param save_path:
    :return:
    """
    list_config = []
    for key in dict_data:
        info = "conf.{}={}".format(key, dict_data[key])
        print(info)
        list_config.append(info)
    if save_path is not None:
        with open(save_path, "w") as f:
            for info in list_config:
                f.writelines(info + "\n")


def read_pair_data(filename, split=True):
    '''
    read pair data,data:[image1.jpg image2.jpg 0]
    :param filename:
    :param split:
    :return:
    '''
    content_list = read_data(filename)
    if split:
        content_list = np.asarray(content_list)
        faces_list1 = content_list[:, :1].reshape(-1)
        faces_list2 = content_list[:, 1:2].reshape(-1)
        # convert to 0/1
        issames_data = np.asarray(content_list[:, 2:3].reshape(-1), dtype=np.int)
        issames_data = np.where(issames_data > 0, 1, 0)
        faces_list1 = faces_list1.tolist()
        faces_list2 = faces_list2.tolist()
        issames_data = issames_data.tolist()
        return faces_list1, faces_list2, issames_data
    return content_list


def check_files(files_list, sizeTh=1 * 1024, isRemove=False):
    ''' 去除不存的文件和文件过小的文件列表
    :param files_list:
    :param sizeTh: 文件大小阈值,单位：字节B，默认1000B ,33049513/1024/1024=33.0MB
    :param isRemove: 是否在硬盘上删除被损坏的原文件
    :return:
    '''
    i = 0
    while i < len(files_list):
        path = files_list[i]
        # 判断文件是否存在
        if not (os.path.exists(path)):
            print(" non-existent file:{}".format(path))
            files_list.pop(i)
            continue
        # 判断文件是否为空
        f_size = os.path.getsize(path)
        if f_size < sizeTh:
            print(" empty file:{}".format(path))
            if isRemove:
                os.remove(path)
                print(" info:----------------remove image_dict:{}".format(path))
            files_list.pop(i)
            continue
        i += 1
    return files_list


def get_files_id(file_list):
    """
    :param file_list:
    :return:
    """
    image_idx = []
    for path in file_list:
        basename = os.path.basename(path)
        id = basename.split(".")[0]
        image_idx.append(id)
    return image_idx


def get_loacl_eth2():
    '''
    想要获取linux设备网卡接口，并用列表进行保存
    :return:
    '''
    eth_list = []
    os.system("ls -l /sys/class/net/ | grep -v virtual | sed '1d' | awk 'BEGIN {FS=\"/\"} {print $NF}' > eth.yaml")
    try:
        with open('./eth.yaml', "r") as f:
            for line in f.readlines():
                line = line.strip()
                eth_list.append(line.lower())
    except Exception as e:
        print(e)
        eth_list = []
    return eth_list


def get_loacl_eth():
    '''
    想要获取linux设备网卡接口，并用列表进行保存
    :return:
    '''
    eth_list = []
    cmd = "ls -l /sys/class/net/ | grep -v virtual | sed '1d' | awk 'BEGIN {FS=\"/\"} {print $NF}'"
    try:
        with os.popen(cmd) as f:
            for line in f.readlines():
                line = line.strip()
                eth_list.append(line.lower())
    except Exception as e:
        print(e, "can not found eth,will set default eth is:eth0")
        eth_list = ["eth0"]
    if not eth_list:
        eth_list = ["eth0"]
    return eth_list


def merge_files(files_list):
    """
    合并文件列表
    :return:
    """
    content_list = []
    for file in files_list:
        data = read_data(file)

    return content_list


def multi_thread_task(content_list, func, num_processes=4, remove_bad=False, Async=True, **kwargs):
    """
    多线程处理content_list的数据
    Usage:
        def task_fun(item, save_root):
            '''
            :param item: 对应content_list的每一项item
            :param save_root: 对应kwargs
            :return:
            '''
            pass
        multi_thread_task(content_list,
                          func=task_fun,
                          num_processes=num_processes,
                          remove_bad=remove_bad,
                          Async=Async,
                          save_root=save_root)
    =====================================================
    :param content_list: content_list
    :param func: func：task function
    :param num_processes: 开启线程个数
    :param remove_bad: 是否去除下载失败的数据
    :param Async:是否异步
    :param kwargs:需要传递给func的相关参数
    :return: 返回图片的存储地址列表
    """
    from multiprocessing.pool import ThreadPool
    # 开启多线程
    pool = ThreadPool(processes=num_processes)
    thread_list = []
    for item in content_list:
        if Async:
            out = pool.apply_async(func=func, args=(item,), kwds=kwargs)  # 异步
        else:
            out = pool.apply(func=func, args=(item,), kwds=kwargs)  # 同步
        thread_list.append(out)

    pool.close()
    pool.join()
    # 获取输出结果
    dst_content_list = []
    if Async:
        for p in thread_list:
            image = p.get()  # get会阻塞
            dst_content_list.append(image)
    else:
        dst_content_list = thread_list
    if remove_bad:
        dst_content_list = [i for i in dst_content_list if i is not None]
    return dst_content_list


if __name__ == '__main__':
    parent = "/media/dm/dm1/git/python-learning-notes/dataset/dataset"
    dir_list = getFilePathList(parent)
    dir_list1 = get_sub_paths(dir_list, parent)
    print(dir_list1)
