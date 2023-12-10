# Author - R Ellingham
# Date - 30 Nov 2023
# Descr - Use blob separation function from PyMesh package through docker to do blob separation externally then save data 
#   in a pkl file for further processing

#   cmdline docker cmd for interactive terminal with access to pkl file directory:
# C:\Users\rel80>docker run -it -v "/c/Users/rel80/OneDrive - University of Canterbury/Postgrad/6. Projects/2. Pressure sensor array/1.0 Code://external" pymesh/pymesh sh -c "cd ../external && python"

#     cmdline docker cmd to run this program:
# os.system("docker run -v \"/c/Users/rel80/OneDrive - University of Canterbury/Postgrad/6. Projects/2. Pressure sensor array/1.0 Code://external\" pymesh/pymesh sh -c \"cd ../external && python /external/eit_analysis/DOCKER_fem_blob_separator.py\"")


## IMPORT LIBS
import numpy as np
import os
import pickle as pkl
import pymesh

## LOAD FUNCS
def load_recon(elem_data, nodes, elems, filename, recon_timestamp_s=[], rtot=[]):
    '''
    Load reconstruction data as compressed .pkl file
    
    note: if loading 'rtot' must also load 'recon_timestamp_s'
    '''
    with open(filename+".pkl","rb") as fp:
        elem_data = pkl.load(fp)
        nodes = pkl.load(fp)
        elems = pkl.load(fp)
        if len(recon_timestamp_s) > 0:
            recon_timestamp_s = pkl.load(fp)
        if len(rtot) > 0:
            rtot = pkl.load(fp)
    
    return elem_data,nodes,elems,recon_timestamp_s,rtot

# Area of a triangle function given 3 points 
A_tri = lambda p1,p2,p3: 1/2 * abs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))

def fem_circ_mask(elem_data_arr, nodes, elems, circ_xyz_loc_mm, circ_rad_mm, estimate_area=False):
    '''
    Only selects FEM data within the specified circle area and sets rest of FEM image to zero.
        elem_data_arr: EIT image data
        nodes: FEM node coordinates
        elems: FEM element nodes
        xyz_loc: Coordinates of the force applicator (only concerned with x and y coordinates)
        circ_rad: Circle radius to mask
    '''
    elem_data_arr_circ_msk = elem_data_arr.copy()
    num_elems = len(elem_data_arr_circ_msk)
    area_sum = 0
    for elem in range(len(elem_data_arr)):
        elem_CoM = np.sum(np.transpose(nodes[elems[elem]]),1)/len(elems[elem])
        # print(f"elem dist = {np.linalg.norm(elem_CoM - circ_xyz_loc_mm)}")
        if np.linalg.norm(elem_CoM - circ_xyz_loc_mm) > circ_rad_mm:
            elem_data_arr_circ_msk[elem] = 0
            num_elems -= 1
        elif estimate_area:
            print()
            px = [nodes[elems[elem]][p] for p in range(3)]
            area_sum += A_tri(px[0],px[1],px[2])
    area_sum = area_sum if area_sum else None
    print(f'Num elems in mask = {num_elems}/{len(elem_data_arr)}. Area = {area_sum}mm^2')
    return elem_data_arr_circ_msk

def thresh_data(data_in, thresh_lo=None, thresh_hi=None):
    threshd_data = np.zeros(data_in.shape)
    if thresh_lo:
        threshd_data += (data_in < thresh_lo) * data_in
    if thresh_hi:
        threshd_data += (data_in > thresh_hi) * data_in
    return threshd_data

def save_mesh_blobs(elem_data_blob_arr, nodes_blob_arr, elems_blob_arr, filename):
    '''
    Save mesh data as compressed .pkl file

    length of elem_data_blob_arr determines the amount of blobs 
    '''
    with open(filename+".pkl","wb") as fp:
        pkl.dump(elem_data_blob_arr,fp)
        pkl.dump(nodes_blob_arr,fp)
        pkl.dump(elems_blob_arr,fp)
    return 0

# INIT PARAMS AND RUN CODE
def main(eit_data_file, raw_data_dir, frame):
    dia_DUT_mm = 100

    elem_data_arr, nodes_arr, elems_arr, _, rtot_arr, nodes, elems = [],[],[],[],[],[],[]

    elem_data_arr, nodes_arr, elems_arr, _, rtot_arr = load_recon(elem_data_arr, nodes, elems, f"{raw_data_dir}/{eit_data_file}_recon")
    nodes_arr_mm = np.array(nodes_arr) * dia_DUT_mm/2 
    nodes = nodes_arr_mm[frame]
    elems = elems_arr[frame]

    ## Threshold data using optimal threshold % found in section B - 2 - a. i.e. 85%
    threshold = 0.85 # percentage threshold mask
    elem_data_arr_th = thresh_data(elem_data_arr[frame], threshold*min(elem_data_arr[frame]), 0)

    # remove unwanted elements
    elems_redu0 = np.reshape(np.transpose(elems)[0],np.shape(abs(elem_data_arr_th) > 0))[abs(elem_data_arr_th) > 0]
    elems_redu1 = np.reshape(np.transpose(elems)[1],np.shape(abs(elem_data_arr_th) > 0))[abs(elem_data_arr_th) > 0]
    elems_redu2 = np.reshape(np.transpose(elems)[2],np.shape(abs(elem_data_arr_th) > 0))[abs(elem_data_arr_th) > 0]
    elems_redu = np.transpose(np.array([elems_redu0, elems_redu1, elems_redu2]))
    elem_data_redu = elem_data_arr_th[abs(elem_data_arr_th) > 0]

###### ONLY NEED THIS SECTION TO BE DONE IN DOCKER >> ########

    ## Pymesh processing section

    # create pymesh mesh
    pm_mesh = pymesh.form_mesh(nodes, elems_redu)
    pm_mesh.add_attribute("conductivity")
    pm_mesh.set_attribute("conductivity", elem_data_redu)

    # separate pymesh mesh
    pm_mesh_sep = pymesh.separate_mesh(pm_mesh)

    # turn back into a raw mesh format ready for manipulation and plotting in the EIT time series analysis notebook
    elem_data_blob_arr = [blob.get_attribute('conductivity') for blob in pm_mesh_sep]
    nodes_blob_arr = [blob.vertices for blob in pm_mesh_sep]
    elems_blob_arr = [blob.faces for blob in pm_mesh_sep]

    filename_path = os.path.join('.', f"{eit_data_file}_blobs_frame{frame}")

    save_mesh_blobs(elem_data_blob_arr, nodes_blob_arr, elems_blob_arr, filename_path)

###### ^^ ONLY NEED THIS SECTION TO BE DONE IN DOCKER ########

    # TODO: determine 'best' blob!

    print("DONE!")

    return 0

if __name__ == "__main__":
    import sys

    filename = sys.argv[1]
    input_file_path = sys.argv[2]
    frame = int(sys.argv[3])


    main(filename, input_file_path, frame)

