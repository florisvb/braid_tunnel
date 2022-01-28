#!/usr/bin/env python

###
#A script meant to take a braidz file and a .hdf5 from a standard windtunnel triggering experiment and performs the appropriate filtering steps, only flies that triggered a flash, flie trajectories of appropriate length, durations that travelled a minimum distance etc.  
#Performs adding a collum that is "unique_id" becuase when many data sets are combine it is possible for the braidobject Id assigning function to give two flies the same id
#creates a "Flash_frame" column that is true if the lights flashed at that time
#creates a "duration" column that assigns the flash duration value to that object id
#creates a "flash_time" column that is the transformed real time either before or after the flash normalized around 0 where the flash occored (i.e. where "Flash frame==True")
#also assigns an orientation 'u' or 'd' (upwind or downwind) based on flight parameters at the time of flashing
###


import pandas as pd 
import numpy as np 
import argparse
import h5py
import os
from braid_analysis import braid_filemanager
from braid_analysis import braid_slicing
from braid_analysis import braid_analysis_plots
import zipfile
import urllib.request

#########
#Setup command line arguments here for taking in a .braidz an .hdf5 file
#
#
#
#
############
parser = argparse.ArgumentParser()
parser.add_argument('braid_filename', type=str, help="full path to the braid file of interest")
parser.add_argument('trigger_filename', type=str, help= 'full path the the trigger bag hdf5')
args = parser.parse_args()
braid_handle = str(args.braid_filename)
trigger_handle = str(args.trigger_filename)
#Function that can take a hdf5 and convert it into a data frame
def get_pandas_dataframe_from_uncooperative_hdf5(filename, key='first_key'):
    f = h5py.File(filename,'r')
    all_keys = list(f.keys())
    if key == 'first_key':
#         print('Dataset contains these keys: ')
        print(all_keys)
        key = all_keys[0]
#         print('Using only the first key: ', key)
    data = f[key][()]
    dic = {}
    for column_label in data.dtype.fields.keys():
        dic.setdefault(column_label, data[column_label])
    df = pd.DataFrame(dic)
    return df


#Create a filtering function that takes in the braid
#data frame and does standard filtering
def do_filtering(braid_file, trigger_df):
	"""This Function performans standard filtering for trajectories with the proper length, and also for trajectories that caused an triggering event"""
	df1 = braid_filemanager.load_filename_as_dataframe_3d(braid_file)
	long_obj_ids1 = braid_slicing.get_long_obj_ids_fast_pandas(df1, length=50)
	df1= df1[df1.obj_id.isin(long_obj_ids1)]
	middle1 = braid_slicing.get_middle_of_tunnel_obj_ids_fast_pandas(df1)
	df1 = df1[df1.obj_id.isin(middle1)]
	far1 = braid_slicing.get_trajectories_that_travel_far(df1, xdist_travelled=0.1)
	df1 =df1[df1.obj_id.isin(far1)]
	df1=df1[df1.obj_id.isin(trigger_df.data_1)]
	return df1

##Because object ID's can have the same number from one experiment to another
#here we create a function that gives each object in the data frame a truly Unique_id
#so that when data from multiple experiments are combined there's no chance of accidentally having two trajectories
#with the same obj_id
def assign_unique_id(braid_df, handle_):
	"""This Function adds a unique ID column to the braid data frame"""
	braid_df['obj_id_unique']=braid_df['obj_id'].apply(lambda x: str(x)+ '_' + handle_)
	return braid_df
def assign_duration_value(braid_df, trigger_df):
	'''Creates two keys in the data frame, one for the 'duration' of the flash (specific for the random flash duration experiments' and one for the Flash_bool, which is True where a flash occured, this is helpful for normalizing the timing of things'''
	braid_df['Flash_bool'] = braid_df['frame'].isin(trigger_df['data_2'].to_list())
	braid_df['duration']= ''
	braid_to_concat =[]
	for i in braid_df['obj_id'].unique():
		dummy_df = braid_df[braid_df['obj_id']==i]
		ind = np.where(trigger_df['data_1']==i)[0][0]
		val = trigger_df['data_4'].iloc[ind]
		dummy_df['duration']=val
		braid_to_concat.append(dummy_df)

	stamped_df = pd.concat(braid_to_concat)
	return stamped_df
#Put everything together
def compound_it_all(braid_file, trigger_file):
	trigger_df = get_pandas_dataframe_from_uncooperative_hdf5(trigger_file)
	filtered_df = do_filtering(braid_handle, trigger_df)
	uniq_id_prefix= braid_handle.split('.')[0]
	uniq_id_prefix = uniq_id_prefix.split('/')[-1]	
	filtered_df = assign_unique_id(filtered_df, uniq_id_prefix)
	filtered_df = assign_duration_value(filtered_df, trigger_df)
	filtered_df.to_csv(uniq_id_prefix + '_trimmed.csv')

compound_it_all(braid_handle, trigger_handle)

###AD FLASH BOOL CALCULATOR 