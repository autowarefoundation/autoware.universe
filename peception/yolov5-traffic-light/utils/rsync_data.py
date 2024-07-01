# -*- coding: utf-8 -*-
import os
import sysrsync
import datetime


def get_polyaxon_output():
    from polyaxon_client.tracking import get_outputs_path
    polyaxon_output = get_outputs_path()
    return polyaxon_output


def polyaxon_root(data_node, dir):
    from polyaxon_client.tracking import get_data_paths, get_outputs_path
    out = os.path.join(get_data_paths()[data_node], dir)
    print("root:{}====>{}".format(dir, out))
    return out


def get_polyaxon_dataroot(data_node="ceph", dir=""):
    """
    :param data_node: data_node APT数据节点,ceph or newceph
    :param dir: str or list
    :return:
    """
    if isinstance(dir, str):
        dir = polyaxon_root(data_node, dir)
    elif isinstance(dir, list):
        dir = [polyaxon_root(data_node, d) for d in dir]
    else:
        raise Exception("Error:{}".format(dir))
    return dir


def polyaxon_env(data_root, val_root, val_dataset, update=False, use_local=True):
    """
    # 训练机器本地盘路径
    host_path = get_data_paths()['host-path']
    :param data_root:
    :param val_root:
    :param val_dataset:
    :param update : data
    :return:
    """
    from polyaxon_client.tracking import get_data_paths, get_outputs_path
    print("The environment is polyaxon")
    # polyaxon_dataroot = os.path.join(get_data_paths()["data-pool"], 'FaceData')  # upload data to TFR file
    host_path = os.path.join(get_data_paths()['host-path'], "FaceData")
    # src_path = os.path.join(get_data_paths()['ssd'], 'FaceData')  # upload data to TFR file
    src_path = os.path.join(get_data_paths()['ceph'], 'FaceData')  # upload data to TFR file
    polyaxon_output = get_outputs_path()

    if isinstance(data_root, str):
        data_root = [data_root]
    if use_local:
        dst_data_root = [os.path.join(host_path, dataset) for dataset in data_root]
        # sync train data
        for i, (image_root, name) in enumerate(zip(dst_data_root, data_root)):
            if update or not os.path.exists(image_root):
                dst_data_root[i] = rsync(src_path, host_path, name)

        # sync val data
        dst_val_root = os.path.join(host_path, val_root)
        # file_processing.remove_dir(dst_val_root)
        for val_name in val_dataset:
            val_name = "{}.bin".format(val_name)
            local_val_path = os.path.join(dst_val_root, val_name)
            if update or not os.path.exists(local_val_path):
                val_src_root = os.path.join(src_path, val_root)
                local_val_path = rsync(val_src_root, dst_val_root, val_name)
    else:
        dst_data_root = [os.path.join(src_path, dataset) for dataset in data_root]
        dst_val_root = os.path.join(src_path, val_root)

    return dst_data_root, dst_val_root, polyaxon_output


def rsync_data():
    """
    rsync dataset
    :return:
    """
    from polyaxon_client.tracking import get_data_paths, get_outputs_path

    source = os.path.join(get_data_paths()['ceph'], 'FaceData')  # upload data to TFR file
    destination = os.path.join(get_data_paths()['host-path'], "FaceData")
    if not os.path.exists(destination):
        os.makedirs(destination)
    print("copy data from:{}".format(source))
    print("destination   :{}".format(destination))
    print("rsync data ...")
    start = datetime.datetime.now()
    sysrsync.run(source=source, destination=destination, options=['-a'])
    end = datetime.datetime.now()
    print("rsync data done,run time:{}".format(end - start))


def rsync(src_root, host_root, name):
    """
    rsync dataset
    :param src_root:
    :param host_root:
    :param name:
    :return:
    """
    source = os.path.join(src_root, name)  # upload data to TFR file
    destination = os.path.join(host_root, name)
    if not os.path.exists(host_root):
        os.makedirs(host_root)
    print("copy data from:{}".format(source))
    print("destination   :{}".format(destination))
    print("rsync data ...")
    start = datetime.datetime.now()
    sysrsync.run(source=source, destination=destination, options=['-a'])
    end = datetime.datetime.now()
    print("rsync data done,run time:{}".format(end - start))
    return destination


def rsync_test(name):
    """
    rsync data test
    :param name:
    :return:
    """
    source = os.path.join('utils', name)  # upload data to TFR file
    destination = os.path.join('FaceData', name)
    if not os.path.exists(destination):
        os.makedirs(destination)
    print("copy data from:{}".format(source))
    print("destination   :{}".format(destination))
    print("rsync data ...")
    start = datetime.datetime.now()
    sysrsync.run(source=source, destination=destination, options=['-a'])
    end = datetime.datetime.now()
    print("rsync data done,run time:{}".format(end - start))
    return destination


if __name__ == "__main__":
    val_src_root = "/media/dm/dm/FaceRecognition/torch-Face-Recognize-Pipeline/data/val/"
    local_val_root = "/media/dm/dm/FaceRecognition/torch-Face-Recognize-Pipeline/data/val1/val"
    val_name = "X4"
    local_val_path = rsync(val_src_root, local_val_root, val_name)
