from __future__ import absolute_import

import os
import errno
import logging


def mkdir_p(dir_path):
    try:
        os.makedirs(dir_path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

def list_file(dir_path, ext=''):
    result = []
    for file in os.listdir(dir_path):
        if file.endswith(ext):
            result.append(file)
    return result

def isfile(fname):
    return os.path.isfile(fname)

def isdir(dirname):
    return os.path.isdir(dirname)

def join(path, *paths):
    return os.path.join(path, *paths)

def read_lines(file_path):
    '''
    Return lines of a text file as a list of strings
    '''
    with open(file_path, 'r') as f:
        content = f.readlines()
    return [x.strip() for x in content]

def setup_logger(name='', log_file='', level=logging.INFO,
    is_formatter=False, is_file=True, is_stream=False):
    """Function setup as many loggers as you want"""
    logger = logging.getLogger(name)
    logger.setLevel(level)

    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')

    if is_file:
        handler = logging.FileHandler(log_file)
        if is_formatter:
            handler.setFormatter(formatter)
        logger.addHandler(handler)

    if is_stream:
        stream_handler = logging.StreamHandler()
        if is_formatter:
            stream_handler.setFormatter(formatter)
        logger.addHandler(stream_handler)

    return logger


def write_args(logger, args):
    args_str = 'Arguments \n'
    for arg in vars(args):
        args_str += '    {}: {}\n'.format(arg, getattr(args, arg))
    logger.info(args_str)


class AverageMeter(object):
    """Computes and stores the average and current value"""
    def __init__(self, name, fmt=':f'):
        self.name = name
        self.fmt = fmt
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count

    def __str__(self):
        fmtstr = '{name} {val' + self.fmt + '} ({avg' + self.fmt + '})'
        return fmtstr.format(**self.__dict__)


class ProgressMeter(object):
    def __init__(self, num_batches, meters, prefix=""):
        self.batch_fmtstr = self._get_batch_fmtstr(num_batches)
        self.meters = meters
        self.prefix = prefix

    def display(self, batch):
        entries = [self.prefix + self.batch_fmtstr.format(batch)]
        entries += [str(meter) for meter in self.meters]
        return '\t'.join(entries)

    def _get_batch_fmtstr(self, num_batches):
        num_digits = len(str(num_batches // 1))
        fmt = '{:' + str(num_digits) + 'd}'
        return '[' + fmt + '/' + fmt.format(num_batches) + ']'
