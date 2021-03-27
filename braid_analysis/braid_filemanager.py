import sys
import zipfile
import urllib.request # requires Python 3
import io
import pandas as pd

def open_filename_or_url(filename_or_url):
    '''
    Opens filename as object for reading.

    Inputs
    ------
    filename_or_url -- (str) filename

    '''
    parsed = urllib.parse.urlparse(filename_or_url)
    is_windows_drive = len(parsed.scheme) == 1
    if is_windows_drive or parsed.scheme=='':
        # no scheme, so this is a filename.
        fileobj_with_seek = open(filename_or_url,mode='rb')
    else:
        # Idea for one day: implement HTTP file object reader that implements
        # seek using HTTP range requests.
        fileobj = urllib.request.urlopen(filename_or_url)
        fileobj_with_seek = io.BytesIO(fileobj.read())
    return fileobj_with_seek

def load_filename_as_dataframe_2d(filename_or_url, frame_range=None):
    '''
    Returns the 2d data from a braidz file as a pandas dataframe.

    Inputs
    ------
    filename_or_url ------ (str) filename
    frame_range  --------- (list of ints) [first frame, last frame]

    Returns
    -------
    data2d_distorted_df -- (dataframe)
    
    '''
    fileobj = open_filename_or_url(filename_or_url)

    with zipfile.ZipFile(file=fileobj, mode='r') as archive:
        cam_info_df = pd.read_csv(
            archive.open('cam_info.csv.gz'),
            comment="#",
            compression='gzip')

        camn2camid = {}
        for i, row in cam_info_df.iterrows():
            camn2camid[row['camn']] = row['cam_id']

        cam_ids = list(cam_info_df['cam_id'].values)
        cam_ids.sort()
        data2d_distorted_df = pd.read_csv(
            archive.open('data2d_distorted.csv.gz'),
            comment="#",
            compression='gzip')
    
    if frame_range is None:
        return data2d_distorted_df
    else:
        return data2d_distorted_df.query('frame > ' + str(frame_range[0]) + \
                                         ' and frame < ' + str(frame_range[-1]))

def load_filename_as_dataframe_3d(filename_or_url, frame_range=None):
    '''
    Returns the 3d data from a braidz file as a pandas dataframe.

    Inputs
    ------
    filename_or_url ------ (str) filename
    frame_range  --------- (list of ints) [first frame, last frame]

    Returns
    -------
    df -- (dataframe)
    
    '''

    fileobj = open_filename_or_url(filename_or_url)

    with zipfile.ZipFile(file=fileobj, mode='r') as archive:
        df = pd.read_csv(
            archive.open('kalman_estimates.csv.gz'),
            comment="#",
            compression='gzip')
    
    if frame_range is None:
        return df
    else:
        return df.query('frame > ' + str(frame_range[0]) + \
                        ' and frame < ' + str(frame_range[-1]))