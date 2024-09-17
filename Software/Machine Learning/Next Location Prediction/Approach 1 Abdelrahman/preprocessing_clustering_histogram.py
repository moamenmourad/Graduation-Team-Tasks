import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans


mfloc_factor = 30

print("Reading dataset rows")
df=pd.read_csv("dataset.csv", na_filter=False)

print('\ndataset sample ============>\n')
print(df.head())


def process_on_users_events(df_p):
	df=[]
	for ev in df_p.to_numpy() :
		if ev[0][0].isalpha() :
			continue
		u_dic={}
		u_dic['user_id']=ev[0].split('_')[2]
		date_arr=ev[1].split(' ')[0].split('-')
		u_dic['time']=ev[1]
		u_dic['year']=date_arr[0]
		u_dic['mon']=date_arr[1]
		u_dic['day']=date_arr[2]
		time_arr=ev[1].split(' ')[1].split(':')
		u_dic['hour']=time_arr[0]
		u_dic['min']=time_arr[1]
		u_dic['sec']=time_arr[2].split('.')[0]
		u_dic['location']=ev[2][2:-2].split(', ')
		u_dic['lat']=ev[2][2:-2].split(', ')[0]
		u_dic['lon']=ev[2][2:-2].split(', ')[1]
		df.append(u_dic)
	return pd.DataFrame(df)
	
df=process_on_users_events(df)
df=df.sort_values(by=['user_id','time'])

print('\n\ndataset sample after processing and sorting ============>\n')
print(df.head())

n_clusters = 10

cluster = KMeans(n_clusters)
location_cluster = cluster.fit_predict(df[['lat', 'lon']])

df['loc_c'] = location_cluster


print('\n\ndataset sample after clustering ============>\n')
print(df.head())


print('\n\nAll possible Locations ============>\n')
location_set=list(set(map(tuple,df['location'])))
print(location_set)

print('\n\nAll Location Clusters ============>\n')
location_c_set=list(set(df['loc_c']))
print(location_c_set)

df.nunique()




print('\n\nSaving dataset to CSV File')
df.to_csv('processed_dataset.csv',index=False)

print('\n\nSelecting Users Locations')

i=-1
user_id_temp=0

df_n=[]

for row in df.to_numpy() :
	if row[0] == user_id_temp :
		df_n[i].append([row[0],[row[-2],row[-1]]])
	else:
		user_id_temp=row[0]
		df_n.append([[row[0],[row[-2],row[-1]]]])
		i=i+1

print('\n\ndataset sample no.1 after selecting users locations ============>\n')
print(df_n[0])



df_mfloc = []
df_loc = []
events_count=[]


print('\n\nFormating user locations ============>\n')
for i in range(0,len(df_n)):
	print("\rWork on collection no : "+str(i)+" of " +str(len(df_n)) + " Having user_Id = " + str(df_n[i][0][0]),end='',flush=True)
	
	u_events=df_n[i]
	if len(u_events) > mfloc_factor :
		events_count.append(len(u_events))
		df_mfloc.append(u_events[0][0])
		df_loc.append(u_events[0][0])
	else :
		df_loc.append(u_events[0][0])


print('\n\nLocations Count Range for Each User_ID In Most Frequent sample ============>\n')
print(min(events_count),max(events_count))

print('\n\nMost Frequent Locations ID sample')
print(df_mfloc[0:5])
print('\nNormal Locations ID sample')
print(df_loc[0:5])

plt.hist(events_count,bins=range(1,len(events_coutn)))

plt.title('Locations Histogram')
plt.xlabel('User_id')
plt.ylabel('Locations Count')
plt.show()
	


