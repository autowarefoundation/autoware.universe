# -*- coding: utf-8 -*-
"""
# --------------------------------------------------------
# @Project: torch-anti-spoofing-pipeline
# @Author : panjq
# @E-mail : pan_jinquan@163.com
# @Date   : 2020-06-02 16:00:47
# --------------------------------------------------------
"""
import torch
import random
import os
import numpy as np
from collections import OrderedDict
from collections.abc import Iterable


def set_env_random_seed(seed=2020):
    """
    :param seed:
    :return:
    """
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)  # if you are using multi-GPU.
    # torch.backends.cudnn.benchmark = False
    # torch.backends.cudnn.deterministic = True


def get_device():
    """
    返回当前设备索引
    torch.cuda.current_device()
    返回GPU的数量
    torch.cuda.device_count()
    返回gpu名字，设备索引默认从0开始
    torch.cuda.get_device_name(0)
    cuda是否可用
    torch.cuda.is_available()
    ==========
    CUDA_VISIBLE_DEVICES=4,5,6 python train.py

    Usage:
    gpu_id = get_device()
    model = build_model()
    model.cuda()
    model = torch.nn.DataParallel(model, device_ids=gpu_id)
    ...
    :return:
    """
    gpu_id = list(range(torch.cuda.device_count()))
    return gpu_id


def print_model(model):
    """
    :param model:
    :return:
    """
    for k, v in model.named_parameters():
        # print(k,v)
        print(k)


def freeze_net_layers(net):
    """
    https://www.zhihu.com/question/311095447/answer/589307812
    example:
        freeze_net_layers(net.base_net)
        freeze_net_layers(net.source_layer_add_ons)
        freeze_net_layers(net.extras)
    :param net:
    :return:
    """
    # for param in net.parameters():
    #     param.requires_grad = False
    for name, child in net.named_children():
        # print(name, child)
        for param in child.parameters():
            param.requires_grad = False


def load_state_dict(model_path, module=True):
    """
    Usage:
        model=Model()
        state_dict = torch_tools.load_state_dict(model_path, module=False)
        model.load_state_dict(state_dict)
    :param model_path:
    :param module:
    :return:
    """
    state_dict = None
    if model_path:
        print('=> loading model from {}'.format(model_path))
        state_dict = torch.load(model_path, map_location=torch.device('cpu'))
        if module:
            state_dict = get_module_state_dict(state_dict)
    else:
        print("Error:no model file:{}".format(model_path))
        exit(0)
    return state_dict


def get_module_state_dict(state_dict):
    """
    :param state_dict:
    :return:
    """
    # 初始化一个空 dict
    new_state_dict = OrderedDict()
    # 修改 key，没有module字段则需要不del，如果有，则需要修改为 module.features
    for k, v in state_dict.items():
        if k.startswith("module."):
            # k = k.replace('module.', '')
            k = k[len("module."):]
        new_state_dict[k] = v
    return new_state_dict


def load_model_pretrained(model, weight_path):
    r"""Loads pretrianed weights to model.
    Features:只会加载完全匹配的模型参数
        - Incompatible layers (unmatched in name or size) will be ignored.
        - Can automatically deal with keys containing "module.".
    Args:
        model (nn.Module): network model.
        weight_path (str): path to pretrained weights.
    """
    checkpoint = load_state_dict(weight_path)
    if 'state_dict' in checkpoint:
        state_dict = checkpoint['state_dict']
    else:
        state_dict = checkpoint
    model_dict = model.state_dict()
    new_state_dict = OrderedDict()
    matched_layers, discarded_layers = [], []
    for k, v in state_dict.items():
        if k.startswith('module.'):
            k = k[7:]  # discard module.
        if k in model_dict and model_dict[k].size() == v.size():
            new_state_dict[k] = v
            matched_layers.append(k)
        else:
            discarded_layers.append(k)

    model_dict.update(new_state_dict)
    model.load_state_dict(model_dict)
    if len(matched_layers) == 0:
        print('The pretrained weights "{}" cannot be loaded,'
              'please check the key names manually'.format(weight_path))
    else:
        print('Successfully loaded pretrained weights from "{}"'.format(weight_path))
        if len(discarded_layers) > 0:
            print('The following layers are discarded due to unmatched keys or layer size: {}'.format(discarded_layers))
    return model


def plot_model(model, output=None, input_shape=None):
    """
    Usage:
    output = model(inputs)
    vis_graph = make_dot(output, params=dict(model.named_parameters()))
    vis_graph.view()
    =================================================================
    output/input_shape至少已知一个
    :param model:
    :param output:
    :param input_shape: (batch_size, 3, input_size[0], input_size[1])
    :return:
    """
    from torchviz import make_dot
    if output is None:
        output = model_forward(model, input_shape, device="cpu")
    vis_graph = make_dot(output, params=dict(model.named_parameters()))
    vis_graph.view()


def model_forward(model, input_shape, device="cpu"):
    """
    input_shape=(batch_size, 3, input_size[0], input_size[1])
    :param model:
    :param input_shape:
    :param device:
    :return:
    """
    inputs = torch.randn(size=input_shape)
    inputs = inputs.to(device)
    model = model.to(device)
    model.eval()
    output = model(inputs)
    return output


def summary_v2(model, *input_tensors, item_length=26, verbose=False):
    from collections import namedtuple
    summary = []
    ModuleDetails = namedtuple("Layer", ["name", "input_size", "output_size", "num_parameters", "multiply_adds"])
    hooks = []
    layer_instances = {}

    def add_hooks(module):
        def hook(module, input, output):
            class_name = str(module.__class__.__name__)
            instance_index = 1
            if class_name not in layer_instances:
                layer_instances[class_name] = instance_index
            else:
                instance_index = layer_instances[class_name] + 1
                layer_instances[class_name] = instance_index
            layer_name = class_name + "_" + str(instance_index)
            params = 0
            if class_name.find("Conv") != -1 or class_name.find("BatchNorm") != -1 or \
                    class_name.find("Linear") != -1:
                for param_ in module.parameters():
                    params += param_.view(-1).size(0)
            flops = "Not Available"
            if class_name.find("Conv") != -1 and hasattr(module, "weight"):
                flops = (
                        torch.prod(
                            torch.LongTensor(list(module.weight.data.size()))) *
                        torch.prod(
                            torch.LongTensor(list(output.size())[2:]))).item()
            elif isinstance(module, torch.nn.Linear):
                flops = (torch.prod(torch.LongTensor(list(output.size()))) \
                         * input[0].size(1)).item()
            if isinstance(input[0], list):
                input = input[0]
            if isinstance(output, list):
                output = output[0]
            summary.append(
                ModuleDetails(
                    name=layer_name,
                    input_size=list(input[0].size()),
                    output_size=list(output.size()),
                    num_parameters=params,
                    multiply_adds=flops)
            )

        if not isinstance(module, torch.nn.ModuleList) \
                and not isinstance(module, torch.nn.Sequential) \
                and module != model:
            hooks.append(module.register_forward_hook(hook))

    model.eval()
    model.apply(add_hooks)
    space_len = item_length
    model(*input_tensors)
    for hook in hooks:
        hook.remove()
    details = ''
    if verbose:
        details = "Model Summary" + \
                  os.linesep + \
                  "Name{}Input Size{}Output Size{}Parameters{}Multiply Adds (Flops){}".format(
                      ' ' * (space_len - len("Name")),
                      ' ' * (space_len - len("Input Size")),
                      ' ' * (space_len - len("Output Size")),
                      ' ' * (space_len - len("Parameters")),
                      ' ' * (space_len - len("Multiply Adds (Flops)"))) \
                  + os.linesep + '-' * space_len * 5 + os.linesep

    params_sum = 0
    flops_sum = 0
    for layer in summary:
        params_sum += layer.num_parameters
        if layer.multiply_adds != "Not Available":
            flops_sum += layer.multiply_adds
        if verbose:
            details += "{}{}{}{}{}{}{}{}{}{}".format(
                layer.name,
                ' ' * (space_len - len(layer.name)),
                layer.input_size,
                ' ' * (space_len - len(str(layer.input_size))),
                layer.output_size,
                ' ' * (space_len - len(str(layer.output_size))),
                layer.num_parameters,
                ' ' * (space_len - len(str(layer.num_parameters))),
                layer.multiply_adds,
                ' ' * (space_len - len(str(layer.multiply_adds)))) \
                       + os.linesep + '-' * space_len * 5 + os.linesep

    details += os.linesep \
               + "Total Parameters: {:,}".format(params_sum) \
               + os.linesep + '-' * space_len * 5 + os.linesep
    FLOPs = flops_sum / (1024 ** 3)  # 单位GFLOPs
    details += "Total Multiply Adds (For Convolution and Linear Layers only): {:,} GFLOPs".format(FLOPs)
    details += os.linesep + '-' * space_len * 5 + os.linesep
    details += "Number of Layers:" + os.linesep
    for layer in layer_instances:
        details += "{}:{} layers\t\n".format(layer, layer_instances[layer])
    details += '-' * space_len * 5 + os.linesep
    print(details)
    return FLOPs, details


def summary_model(model, batch_size=1, input_size=[112, 112], plot=False, device="cpu"):
    """
    ----This tools can show----
    Total params: 359,592
    Total memory: 47.32MB
    Total MAdd: 297.37MMAdd
    Total Flops: 153.31MFlops
    Total MemR+W: 99.7MB
    ====================================================
    https://www.cnblogs.com/xuanyuyt/p/12653041.html
    Total number of network parameters (params)
    Theoretical amount of floating point arithmetics (FLOPs)
    Theoretical amount of multiply-adds (MAdd MACC) (乘加运算)
    Memory usage (memory)
    MACCs：是multiply-accumulate operations，指点积运算， 一个 macc = 2FLOPs
    FLOPs 的全称是 floating points of operations，即浮点运算次数，用来衡量模型的计算复杂度。
    计算 FLOPs 实际上是计算模型中乘法和加法的运算次数。
    卷积层的浮点运算次数不仅取决于卷积核的大小和输入输出通道数，还取决于特征图的大小；
    而全连接层的浮点运算次数和参数量是相同的。
    ====================================================
    :param model:
    :param batch_size:
    :param input_size:
    :param plot: plot model
    :param device:
    :return:
    """
    from torchsummary import summary
    from torchstat import stat
    inputs = torch.randn(size=(batch_size, 3, input_size[1], input_size[0]))
    inputs = inputs.to(device)
    model = model.to(device)
    model.eval()
    output = model(inputs)
    # 统计模型参数
    # summary(model, input_size=(3, input_size[1], input_size[0]), batch_size=batch_size, device=device)
    # 统计模型参数和计算FLOPs
    # stat(model, (3, input_size[1], input_size[0]))
    # summary可能报错，可使用该方法
    # summary_v2(model, inputs, item_length=26, verbose=True)
    # from thop import profile
    # macs, params = profile(model, inputs=(inputs,))
    # print("Total Flops :{}".format(macs))
    # print("Total params:{}".format(params))
    print("===" * 10)
    print("inputs.shape:{}".format(inputs.shape))
    # print("output.shape:{}".format(output.shape))
    if plot:
        plot_model(model, output)


def torchinfo_summary(model, batch_size=1, input_size=[112, 112], plot=False, device="cpu"):
    """
    ----This tools can show----
    Total params: 359,592
    Total memory: 47.32MB
    Total MAdd: 297.37MMAdd
    Total Flops: 153.31MFlops
    Total MemR+W: 99.7MB
    ====================================================
    https://www.cnblogs.com/xuanyuyt/p/12653041.html
    Total number of network parameters (params)
    Theoretical amount of floating point arithmetics (FLOPs)
    Theoretical amount of multiply-adds (MAdd MACC) (乘加运算)
    Memory usage (memory)
    MACCs：是multiply-accumulate operations，指点积运算， 一个 macc = 2FLOPs
    FLOPs 的全称是 floating points of operations，即浮点运算次数，用来衡量模型的计算复杂度。
    计算 FLOPs 实际上是计算模型中乘法和加法的运算次数。
    卷积层的浮点运算次数不仅取决于卷积核的大小和输入输出通道数，还取决于特征图的大小；
    而全连接层的浮点运算次数和参数量是相同的。
    ====================================================
    :param model:
    :param batch_size:
    :param input_size:
    :param plot: plot model
    :param device:
    :return:
    """
    from torchinfo import summary
    from torchstat import stat
    inputs = torch.randn(size=(batch_size, 3, input_size[1], input_size[0]))
    inputs = inputs.to(device)
    model = model.to(device)
    model.eval()
    output = model(inputs)
    # 统计模型参数
    summary(model, input_size=(batch_size, 3, input_size[1], input_size[0]), device=device)
    # 统计模型参数和计算FLOPs
    stat(model, (3, input_size[1], input_size[0]))
    # summary可能报错，可使用该方法
    # summary_v2(model, inputs, item_length=26, verbose=True)
    # from thop import profile
    # macs, params = profile(model, inputs=(inputs,))
    # print("Total Flops :{}".format(macs))
    # print("Total params:{}".format(params))
    print("===" * 10)
    print("inputs.shape:{}".format(inputs.shape))
    # print("output.shape:{}".format(output.shape))
    if plot:
        plot_model(model, output)
