# visualization.py utility visualization functions.

from email.errors import BoundaryError
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import transforms
from ConcaveHull import ConcaveHull

import numpy as np
import os
import pandas as pd
import open3d as o3d
import numpy as np
import math
from math import sqrt

from matplotlib import rc
rc('font', **{'family':'DejaVu Sans Mono'})
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib as mpl
from matplotlib.collections import PatchCollection
mpl.rc('axes', labelsize=14)
mpl.rc('xtick', labelsize=12)
mpl.rc('ytick', labelsize=12)

from shapely.geometry import Polygon
from shapely.geometry import MultiLineString
from shapely.geometry import LineString
from shapely.geometry import Point, Polygon,LinearRing, MultiPoint
from shapely import affinity
from shapely import geometry
from descartes import PolygonPatch

# Where to save the figures
PROJECT_ROOT_DIR = ".."
IMAGES_PATH = os.path.join(PROJECT_ROOT_DIR, "png_images")
DATA_PATH = os.path.join(PROJECT_ROOT_DIR, "dataset")
XYZ_point_PATH = os.path.join(DATA_PATH, "xyz")
PCD_file_PATH = os.path.join(DATA_PATH, "pcd")
file_output_dir =  os.path.join(PROJECT_ROOT_DIR, "output_dir/surface_fitting_result")
os.makedirs(IMAGES_PATH, exist_ok=True)
os.makedirs(file_output_dir, exist_ok=True)


color_list = ["b", '#90EE90', '#333333', 'c', 'y', 'k']
color_list2 = ['b', 'orange', 'g', 'r', 'c', 'm', 'y', 'k', 'Brown', 'ForestGreen']

GM = (sqrt(5)-1.0)/2.0
W = 8.0
H = W*GM
SIZE = (W, H)

BLUE = '#6699cc'
GRAY = '#999999'
DARKGRAY = '#333333'
YELLOW = '#ffcc33'
GREEN = '#339933'
RED = '#ff3333'
BLACK = '#000000'

COLOR_ISVALID = {
    True: BLUE,
    False: RED,
}

figure_size = (8,8)

def plot_line(ax, ob, color=GRAY, zorder=1, linewidth=3, alpha=1):
    x, y = ob.xy
    ax.plot(x, y, color=color, linewidth=linewidth, solid_capstyle='round', zorder=zorder, alpha=alpha)

def plot_coords(ax, ob, color=GRAY, zorder=1, alpha=1):
    x, y = ob.xy
    ax.plot(x, y, 'o', color=color, zorder=zorder, alpha=alpha)

def color_isvalid(ob, valid=BLUE, invalid=RED):
    if ob.is_valid:
        return valid
    else:
        return invalid

def color_issimple(ob, simple=BLUE, complex=YELLOW):
    if ob.is_simple:
        return simple
    else:
        return complex

def plot_line_isvalid(ax, ob, **kwargs):
    kwargs["color"] = color_isvalid(ob)
    plot_line(ax, ob, **kwargs)

def plot_line_issimple(ax, ob, **kwargs):
    kwargs["color"] = color_issimple(ob)
    plot_line(ax, ob, **kwargs)

def plot_bounds(ax, ob, zorder=1, alpha=1):
    x, y = zip(*list((p.x, p.y) for p in ob.boundary))
    ax.plot(x, y, 'o', color=BLACK, zorder=zorder, alpha=alpha)

def add_origin(ax, geom, origin):
    x, y = xy = affinity.interpret_origin(geom, origin, 2)
    ax.plot(x, y, 'o', color=GRAY, zorder=1)
    ax.annotate(str(xy), xy=xy, ha='center',
                textcoords='offset points', xytext=(0, 8))

def set_limits(ax, x0, xN, y0, yN):
    ax.set_xlim(x0, xN)
    ax.set_xticks(range(x0, xN+1))
    ax.set_ylim(y0, yN)
    ax.set_yticks(range(y0, yN+1))
    ax.set_aspect("equal")

## function for automatically save the diagram/graph into the folder 
def save_fig(fig_id, tight_layout=True, fig_extension="png", resolution=300):
    path = os.path.join(IMAGES_PATH, fig_id + "." + fig_extension)
    print("Saving figure", fig_id)
    if tight_layout:
        plt.tight_layout()
    plt.savefig(path, format=fig_extension, dpi=resolution)


def calculate_polygon_bondary_with_point_clusters(point_clusters, tolerance = 5, lines=False):
    '''
    input: point clusters, a list object
    tolerance: for approximating the boundaries
    lines: plot boundary points or lines
    '''
    num_clusters = len(point_clusters)
    ch = []
    # pts = []
    boundary_points=[]

    plt.figure(figsize=figure_size)
    for i in range(num_clusters):
        # ch_temp = ConcaveHull()
        # pts_temp = point_clusters[i]
        ch.append(ConcaveHull())
        # pts.append(point_clusters[i])
        ch[i].loadpoints(point_clusters[i])
        ch[i].calculatehull(tol = tolerance)
        boundary_points.append(np.vstack(ch[i].boundary.exterior.coords.xy).T)

        plt.scatter(point_clusters[i][:, 0], point_clusters[i][:, 1], cmap=color_list[i], s = 2, alpha = 0.4)
        if lines==True:
            plt.plot(boundary_points[i][:, 0], boundary_points[i][:, 1], 'r')
        else:
            plt.plot(boundary_points[i][:, 0], boundary_points[i][:, 1], 'r.', linewidth=2)

    # xd=[-10.5, 10.5, 10.5,-10.5,-10.5]
    # yd=[-20.3,-20.3, 20.3, 20.3,-20.3]
    xd=[-15.5, 15.5, 15.5,-15.5,-15.5]
    yd=[-15.3,-15.3, 15.3, 15.3,-15.3]

    plt.xlabel('X (mm)', fontsize=20, labelpad=10)
    plt.ylabel('Y (mm)', fontsize=20, labelpad=1)
    # plt.xticks(np.arange(-10, 10, 10))
    # plt.xticks([-10, 0, 10], fontsize = 16) 
    plt.xticks([-20, -10, 0 , 10, 20], fontsize = 16)
    plt.yticks([-20, -10, 0 , 10, 20], fontsize = 16)

    plt.plot(xd,yd,'k');
    # plt.show()

    return boundary_points


def visualize_bulge_area(boundary_points, ext):
    '''
    input:
    ext -- exterior boundary in CCW direction
    boundary points - a list object, containing boundaries for each polygons
    '''
    fig = plt.figure(1, figsize=figure_size, dpi=90)
    ax = plt.axes()
    # define interior boundary list
    interior_boundary = []
    for i in range(len(boundary_points)):
        interior_boundary.append(boundary_points[i][::-1])
    
    polygon = Polygon(ext, interior_boundary)

    for i in range(len(boundary_points)):
        plot_coords(ax, polygon.interiors[i])
        plot_line(ax, ob=polygon.interiors[i], color=BLACK, zorder=5, linewidth=2, alpha=1)

    plot_coords(ax, polygon.exterior)
    plot_line(ax, ob=polygon.exterior, color=BLACK, zorder=1, linewidth=2, alpha=1)
    patch = PolygonPatch(polygon, facecolor=color_isvalid(polygon), 
                        edgecolor=color_isvalid(polygon, valid=BLUE), alpha=0.4, zorder=2)
    ax.add_patch(patch)

    
    plt.xlabel('X (mm)', fontsize=20, labelpad=10)
    plt.ylabel('Y (mm)', fontsize=20, labelpad=1)
    # plt.xticks(np.arange(-10, 10, 10))
    # plt.xticks([-10, 0, 10], fontsize = 16) 
    plt.xticks([-20, -10, 0 , 10, 20], fontsize = 16)
    plt.yticks([-20, -10, 0 , 10, 20], fontsize = 16)
    ax.set_title('Plot of Bulge area Polygon as Holes')
    # plt.show()

    return polygon


def visualize_line_polygon_segmentation(polygon, intersection_points, line_segment_in, line_segment_out):
    fig = plt.figure(1, figsize=figure_size, dpi=90)
    ax = plt.axes()

    #----------------------plot the polygon------------------------------
    plot_line(ax, ob=polygon.exterior, color=BLACK, zorder=5, linewidth=2, alpha=1)
    for i in range(len(polygon.interiors)):
        plot_line(ax, ob=polygon.interiors[i], color=BLACK, zorder=5, linewidth=2, alpha=1)

    #----------------------plot the intersection point------------------------------
    xs = [point.x for point in intersection_points.geoms]
    ys = [point.y for point in intersection_points.geoms]
    ax.plot(xs, ys, 'o', color=RED, zorder=10, alpha=1)

    
    ## --------------------plot intersection lines-------------------------------------------------------------
    for i in range (len(line_segment_in.geoms)):  
        plot_line(ax, ob=line_segment_in.geoms[i], color=RED, zorder=10, linewidth=2, alpha=1)
        
    for i in range (len(line_segment_out.geoms)):  
        plot_line(ax, ob=line_segment_out.geoms[i], color=GREEN, zorder=10, linewidth=2, alpha=1)


    
    patch = PolygonPatch(polygon, facecolor=color_isvalid(polygon), 
                        edgecolor=color_isvalid(polygon, valid=BLUE), alpha=0.5, zorder=2)
    ax.add_patch(patch)

    ax.set_xlim(-11.5, 11.5)
    ax.set_ylim(-22, 22)

    plt.xlabel('X (mm)', fontsize=20, labelpad=10)
    plt.ylabel('Y (mm)', fontsize=20, labelpad=1)
    # plt.xticks(np.arange(-10, 10, 10))
    # plt.xticks([-10, 0, 10], fontsize = 16) 
    plt.xticks([-20, -10, 0 , 10, 20], fontsize = 16)
    plt.yticks([-20, -10, 0 , 10, 20], fontsize = 16)

    # plt.show()


def visualize_grid_polygon_segmentation(polygon, segment_inside_collection, segment_out_collection, intersection_points_collection):
    fig = plt.figure(1, figsize=figure_size, dpi=90)
    ax = plt.axes()
    
    #----------------------plot polygon with boundaries----------------------------------------------------------
    plot_line(ax, ob=polygon.exterior, color=BLACK, zorder=10, linewidth=2.5, alpha=1)
    for i in range(len(polygon.interiors)):
        plot_line(ax, ob=polygon.interiors[i], color=BLACK, zorder=10, linewidth=2.5, alpha=1)

    
    # segment_inside_collection, segment_out_collection, intersection_points_collection
    #----------------------plot the intersection point----------------------------------------------------------
    for intersection_points in intersection_points_collection:
        xs = [point.x for point in intersection_points.geoms]
        ys = [point.y for point in intersection_points.geoms]
    #     ax.plot(xs, ys, 'o', color=RED, zorder=10, alpha=0.7, ms = 3)
        ax.plot(xs, ys, 'o', color=RED, zorder=10, alpha=1)
    # ax.plot(intersection_points, 'o', color=RED, zorder=1, alpha=1)

    
    ## --------------------plot segmented lines-------------------------------------------------------------
    for in_seg in segment_inside_collection:
        for i in range (len(in_seg.geoms)):  
            plot_line(ax, ob=in_seg.geoms[i], color=RED, zorder=10, linewidth=2.5, alpha=1)

            
    for out_seg in segment_out_collection:   
        for i in range (len(out_seg.geoms)):  
            plot_line(ax, ob=out_seg.geoms[i], color=GREEN, zorder=10, linewidth=2.5, alpha=1)
    
    
    patch = PolygonPatch(polygon, facecolor=color_isvalid(polygon), edgecolor=color_isvalid(polygon, valid=BLUE), alpha=0.5, zorder=2)
    ax.add_patch(patch)

    # ax.set_xlim(-11.5, 11.5)
    # ax.set_ylim(-22, 22)
    plt.xlabel('X (mm)', fontsize=20, labelpad=10)
    plt.ylabel('Y (mm)', fontsize=20, labelpad=1)
    # plt.xticks(np.arange(-10, 10, 10))
    # plt.xticks([-10, 0, 10], fontsize = 16) 
    plt.xticks([-20, -10, 0 , 10, 20], fontsize = 16)
    plt.yticks([-20, -10, 0 , 10, 20], fontsize = 16)

    # plt.show()


def rotate_point_cloud(batch_data):
    """ Randomly rotate the point clouds to augument the dataset
        rotation is per shape based along up direction
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, rotated batch of point clouds
    """
    rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
    for k in range(batch_data.shape[0]):
        rotation_angle = np.random.uniform() * 0.3 * np.pi
        cosval = np.cos(rotation_angle)
        sinval = np.sin(rotation_angle)
        rotation_matrix = np.array([[cosval, 0, sinval],
                                    [0, 1, 0],
                                    [-sinval, 0, cosval]])
        shape_pc = batch_data[k, ...]
        rotated_data[k, ...] = np.dot(shape_pc.reshape((-1, 3)), rotation_matrix)
    return rotated_data


def rotate_point_cloud_by_angle(batch_data, rotation_angle):
    """ Rotate the point cloud along up direction with certain angle.
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, rotated batch of point clouds
    """
    rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
    for k in range(batch_data.shape[0]):
        #rotation_angle = np.random.uniform() * 2 * np.pi
        cosval = np.cos(rotation_angle)
        sinval = np.sin(rotation_angle)
        rotation_matrix = np.array([[cosval, 0, sinval],
                                    [0, 1, 0],
                                    [-sinval, 0, cosval]])
        shape_pc = batch_data[k, ...]
        rotated_data[k, ...] = np.dot(shape_pc.reshape((-1, 3)), rotation_matrix)
    return rotated_data


def LinePolygonIntersectionPoints(input_line, polygon):
    '''
    Functionality:
        Return the line and polygon intersection points
    
    Input variable:
        - line: shapely LineString object, example LineString([(x1, y1), (x2, y2)])
        - polygon: shapely Polygon object, contains exterior and interiors vertices (holes)
        
        example: list(polygon.exterior.coords)
             [(-10.5, -20.5), (10.5, -20.5), (10.5, 20.5), (-10.5, 20.5), (-10.5, -20.5)]
        polygon = Polygon(ext, [boundary_points0[::-1], boundary_points1[::-1], boundary_points2[::-1], boundary_points3[::-1], boundary_points4[::-1]])
    
    
    Output:
        - intersection_multipoint: a MultiPoint object containing all the intersection point.

    '''
     
    
    #---------STEP 1:--------------------
    #---Find Intersection Point----------
    #------------------------------------
    ## empty list to store the intersection point, 
    ## a list of Point object
    list_points = [] 
    
    # -------get the boundary points of the polygon ----------------
    polygon_string = polygon.boundary  ## type is MultiLineString

    # iterate each edge lines in the polygon
    for edges in polygon_string.geoms:
        if edges.intersection(input_line): # if intersection point exists
            intersection_point = edges.intersection(input_line) ## intersection_point is a MultiPoints object
#             print (intersection_point)
            # iterate each point in this MultiPoint object
            if intersection_point is not None: # check if it is empty
                for point in intersection_point.geoms:
                    # append the point to the list
                    list_points.append(point)
    
    # convert the point list to a tuple
    list_points = tuple(list_points)
    # create a MultiPoint Object to return all the intersection points
    intersection_multipoint = MultiPoint(list_points)   
    
    # print(intersection_multipoint)
    
    return intersection_multipoint


def sort_MultiPoint(multipoint, ascending = True, axis = 1):
    '''
    This function sort the Shapely MultiPoint Object
    
    Input/argument:
        - multipoint: a shapely multipoint object
        - ascending: Ture or False (descending order)
        - axis 0 or 1 or None, optional, default 1
          (sort the array based on the first or second colum)
          
          
    Return : the multipoint object sorted
    '''
    
    # an temp empty list storing each points' (x,y) coordinate
    list_point_coord = []

    for point in multipoint.geoms:
        list_point_coord.append(point.coords[0]) # append the coordinate to the list

    # convert the list to numpy array
    array_point_coord = np.array(list_point_coord)    
#     print (array_point_coord) 
#     print (array_point_coord.shape)

    if (ascending==True):
        if (axis == 1):
            # sort the array based on the second colum -- ascending order
            sorted_point = array_point_coord[np.argsort(array_point_coord[:, 1])]
        if (axis == 0):
            sorted_point = array_point_coord[np.argsort(array_point_coord[:, 0])]
    else:
        if (axis == 1):
            # sort the array based on the second colum -- descending order
            sorted_point = array_point_coord[np.argsort(array_point_coord[:, 1])[::-1]]
        if (axis == 0):
            sorted_point = array_point_coord[np.argsort(array_point_coord[:, 0])[::-1]]
            
    
    ## Convert the sorted array into the MultiPoint
    list_point_coord = []

    for point in sorted_point:
    #     print (point) ## each point is a list
        point_ = Point (tuple(point)) # convert each point to a Shapely Point object 
        list_point_coord.append(point_)

#     print (list_point_coord)

    sorted_multi_point = MultiPoint(tuple(list_point_coord))
    return sorted_multi_point   

        

def LineInPolygonSegmentation(input_line, input_polygon):
    '''
    Functionality:
        - to ouput the segment of a straight line inside and outside the polygon
        
        
    Input: LineString object representing a straight lines
    Ouput: Segments of lines. InSeg/OutSeg
    
    '''
    
    
    line_segment_in = [] # empty list to store the segment inside polygon
    line_segment_out = [] # empty list to store the segment outside the polygon
    
    
    #---Check if line is moving left-----
    #---to right or vice versa-----------
    #------------------------------------
    if (input_line.coords[0][0] < input_line.coords[1][0]): ### x1 < x2;
        isLeftToRight = True
        isVertical = False
        isRightToLeft = False
    elif ((input_line.coords[0][0] == input_line.coords[1][0])):   ### x1 = x2;
        isLeftToRight = False
        isVertical = True
        isRightToLeft = False
    else:                                                     ## x1 < x2
        isLeftToRight = False
        isVertical = False
        isRightToLeft = True
    
    
    #---------STEP 1:--------------------
    #---Get intersection point-----------
    #------------------------------------
    # a MultiPoint Object
    intersection_points = LinePolygonIntersectionPoints(input_line, input_polygon)
    
    ### check number of intersection points
    num_points = len(intersection_points.geoms)
    
    ## sort the multipoints in certain orders
    if (num_points != 0):
        if isLeftToRight: 
            # sort the points in x axis, ascending order
            intersection_point_sorted = sort_MultiPoint(intersection_points, ascending = True, axis = 0) 
        elif isVertical:
            # sort the points in y axis, ascending order
            intersection_point_sorted = sort_MultiPoint(intersection_points, ascending = True, axis = 1)
        elif isRightToLeft:
            # sort the points in x axis, descending order
            intersection_point_sorted = sort_MultiPoint(intersection_points, ascending = False, axis = 0)
            
    else:
        return None, None
            
    
    number_of_segments = num_points + 1
      
    #----------------------------------------------
    # Divide line into in-segemnts and out-segments
    # ---------------------------------------------
    #----------------------------------------------

    # from the first coordinate 
    currentSegmentIsIn = input_polygon.contains(Point([input_line.coords[0]]))
    
    for seg_point in range(num_points):
        #  LineString([(5, -23), (5, 23)]) object
          
        if (seg_point == 0):
            ## if current intersection point is the first point (closest to x1,y1)
            current_seg_line = LineString([Point([input_line.coords[0]]), 
                                           intersection_point_sorted.geoms[seg_point]])
                                          
        else:   
            current_seg_line = LineString([intersection_point_sorted.geoms[seg_point-1], 
                                           intersection_point_sorted.geoms[seg_point]]) 
                        
            if currentSegmentIsIn:
                # append current segment into the line_segment_in list
                line_segment_in.append(current_seg_line)
            else:
                line_segment_out.append(current_seg_line)
                                          
        currentSegmentIsIn = ~currentSegmentIsIn
    
    
    #-------------------------------------------------------------------------
#     ## add the last segment: (from last intersection point to x2, y2)
#     current_seg_line = LineString([intersection_point_sorted.geoms[seg_point], 
#                                    Point([input_line.coords[1]])]) 
    
#     if currentSegmentIsIn:
#         # append current segment into the line_segment_in list
#         line_segment_in.append(current_seg_line)
#     else:
#         line_segment_out.append(current_seg_line)
    #-------------------------------------------------------------------------
    
        
    line_segment_in = tuple(line_segment_in) 
    line_segment_out = tuple(line_segment_out)
    
    line_segment_in_mutiline = MultiLineString(line_segment_in)
    line_segment_out_mutiline = MultiLineString(line_segment_out)
    # print (line_segment_in_mutiline)
    # print ("\n")
    # print (line_segment_out_mutiline)
    # print ("\n")
    
    return line_segment_in_mutiline, line_segment_out_mutiline




def grid_line_creation(polygon, delta_x = 1, delta_y = 1000, nl = 150):
    '''
    Input variables:
        - delta_x determines the intervals of lines in x direction
        - the dx/dy determins the slope of the lines, larger delta_y gives more vertical lines
        - nl: a hyperparameter determin the grid points
        - polygon: the input polygon
    
    Output:
        - line_elements: a list of lines (LineString objects)
    '''
    
    x_countour = tuple([point.x for point in MultiPoint(polygon.exterior.coords)]) # tuples, x coordinates of the polygon exterior contour
    y_countour = tuple([point.y for point in MultiPoint(polygon.exterior.coords)])
    
    # two empty list for temperarily store variables
    x_extended = []
    y_extended = []

    # this means 0,1,2,.... nl-1, there are nl numbers in total
    for n in range(nl): 
        # floor returns floor of x - the largest integer not greater than x
        xe = math.floor(min(x_countour) / delta_x) * delta_x + n * delta_x
        ye = math.ceil(max(y_countour) / delta_y) * delta_y - n *delta_y
        x_extended.append(xe)
        y_extended.append(ye)
    y_extended.sort() # sort the list in ascending order
    
    
    xa = np.array([x_extended])                 # shape is (1, nl)
    ya = np.ones((1, nl)) * y_extended[-1]      # shape is (1, nl)
    xb = np.ones((1, nl)) * x_extended[0]       # shape is (1, nl)
    yb = np.array([y_extended[::-1]])           # shape is (1, nl)


    xl = np.concatenate((xa, xb)) # shape should be (2, nl)
    yl = np.concatenate((ya, yb)) # shape should be (2, nl)
    
    
    ## create the LineString element based on the xl yl
    line_elements = []

    for i in range (nl):
        # example: line_element = LineString([(5, -23), (5.1, 23)])
        # xl(1,ii), yl(1,ii), xl(2,ii), yl(2,ii)
        line = LineString([(xl[0,i], yl[0,i]), (xl[1,i], yl[1,i])])
        line_elements.append(line)
        
    return line_elements



def visualize_grid_line_generation(line_elements_list, save_fig = False, size=(8, 8)):
    fig = plt.figure(1, figsize=size, dpi=110)

    
    for count,line_elements_items in enumerate(line_elements_list):
        item = count + 1
        ax = fig.add_subplot(2,3,item)
        ## --------------------plot intersection lines-------------------------------------------------------------
        for i in range (len(line_elements_items)):  
            plot_line(ax, ob=line_elements_items[i], color=RED, zorder=10, linewidth=1, alpha=1)

        # ax.set_xlim(-11.5, 11.5)
        # ax.set_ylim(-22, 22)
        # ax.set_xlim(-15.5, 15.5)
        # ax.set_ylim(-15.5, 15.5)
        plt.xlabel('X (mm)', fontsize=20, labelpad=1)
        plt.ylabel('Y (mm)', fontsize=20, labelpad=1)
        # plt.xticks(np.arange(-10, 10, 10))
        # plt.xticks([-10, 0, 10], fontsize = 16) 
        # plt.xticks([-20, -10, 0 , 10, 20], fontsize = 16)
        # plt.yticks([-20, -10, 0 , 10, 20], fontsize = 16)
        # ax.set_title('Grid Line 1 \n delta_x = 1, delta_y = 1000 \n nl = 150')

        plt.tight_layout()
        
        if save_fig:
            save_fig("Step11 - Grid line creation")



def reverse_MultiLineString(multilinestring):
    '''
    This function reverse the Shapely MultiLineString Object
    
    Input/argument:
        - multilinestring: a shapely MultiLineString object
          
    Return : the MultiLineString object reversed
    '''
    
    list_lines = []
    for line in multilinestring.geoms:
        list_lines.append(line)

    list_line_reversed = list_lines[::-1]
    reversed_multilinestring = MultiLineString(tuple(list_line_reversed))
    return reversed_multilinestring


def zig_zag_segmentation(line_elements, polygon_buffered, nl = 150):
    '''
    Input
        - line_elements: a list object, contains a list of created grid lines (each element is a LineString object)
        
    Output:
        - segment_inside_collection: a list object, each element is a MultiLineString object containing the inside polygon portion of line
        - segment_outside_collection: a list, each element is a MultiLineStirng object containing outside portion of the line
        - intersection_points_collection: list, each element is a MultiPoint object containing the intersection point of lines and polygon
    '''
    
    # line_elements = grid_line_creation(polygon, delta_x = 1, delta_y = 1000, nl = 150)
    
    dirr = True
    dirr_out = True
    segment_inside_collection = [] # empty list for the inside portion of the lines
    segment_outside_collection = [] # empty list for the outside portion of the lines
    intersection_points_collection = [] # empty list for all intersection points

    # from 0 to nl-1
    for i in range (nl):
        intersection_points = LinePolygonIntersectionPoints(line_elements[i], polygon_buffered)
        line_segment_in, line_segment_out = LineInPolygonSegmentation(line_elements[i], polygon_buffered)
        
        if (len(intersection_points.geoms) != 0 ):
            intersection_points_collection.append(intersection_points)
#             if (dirr):
#                     segment_inside_collection.append (line_segment_in)
#             else:
#                 segment_inside_collection.append (reverse_MultiLineString(line_segment_in))
#             dirr = ~dirr
        

        # if there is intersection between polygon and line
        if (line_segment_in is not None):
            if (dirr):
                segment_inside_collection.append (line_segment_in)
            else:
                segment_inside_collection.append (reverse_MultiLineString(line_segment_in))
            dirr = ~dirr
            
        if (line_segment_out is not None):
            if (dirr_out):
                segment_outside_collection.append (line_segment_out)
            else:
                segment_outside_collection.append (reverse_MultiLineString(line_segment_out))
            dirr_out = ~dirr_out


                
    return segment_inside_collection, segment_outside_collection, intersection_points_collection


def visualize_DBSCAN(labels, n_clusters_, point_cloud_numpy, core_samples_mask, save = False, show_axis = True):
    plt.figure(figsize=figure_size)
    # plt.figure()
    ax = plt.axes(projection='3d')
    #------------------------------------------------------------------------------
    ## set colors for differnt labels
    # Black removed and is used for noise instead.
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each)
            for each in np.linspace(0, 1, len(unique_labels))]

    point_clusters = []
    cluster = 0 
    while cluster < n_clusters_:
        class_member_mask = (labels == cluster)
        point_clusters.append(point_cloud_numpy[class_member_mask & core_samples_mask])
    #     point_clusters.append(extracted_2D_downsampled[class_member_mask & core_samples_mask])
        cluster = cluster + 1
        
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = (labels == k)
        
        xyz = point_cloud_numpy[class_member_mask & core_samples_mask]
    #     xy = extracted_2D_downsampled[class_member_mask & core_samples_mask]
    #     plt.plot(xy[:, 0], xy[:, 1], '.', markerfacecolor=tuple(col),
    #              markeredgecolor='k', markersize=14)
        ax.scatter3D(xyz[:, 0], xyz[:, 1],xyz[:, 2], '.', facecolor=tuple(col),
                edgecolor='None', s=2)

        # these are the points does not belong to either clusters (noise)
        xyz = point_cloud_numpy[class_member_mask & ~core_samples_mask]
    #     xy = extracted_2D_downsampled[class_member_mask & ~core_samples_mask]
        ax.scatter3D(xyz[:, 0], xyz[:, 1], xyz[:, 2], '.', facecolor=tuple(col),
                edgecolor='k', s=2)
    #-----------------------------------------------------------------------------------------------


    ax.set_xlabel('X (mm)', fontsize=25, labelpad=25)
    ax.set_ylabel('Y (mm)', fontsize=25,labelpad=25)
    ax.set_zlabel('Z (mm)', fontsize=25, labelpad=20);
    ax.set_zticks([0, 0.5, 1, 1.5, 2])
    ax.set_zlim(-1.5, 3)
    # ax.set_xticks([-20, -10, 0, 10, 20])
    # ax.set_yticks([-30, -20, -10, 0, 10, 20, 30])

    for t in ax.zaxis.get_major_ticks(): t.label.set_fontsize(20)
    for t in ax.xaxis.get_major_ticks(): t.label.set_fontsize(20)
    for t in ax.yaxis.get_major_ticks(): t.label.set_fontsize(20)

    # to hide the 3D axes
    if not show_axis:
        ax._axis3don = False

    # plt.title('3D Point cloud and reference plane')
    ax.view_init(elev=20,azim=-20) #rotate the graph

    if save:
        save_fig("DBSCAN_visualization")


def visualize_2D_DBSCAN(labels, n_clusters_, point_cloud_numpy, core_samples_mask, save = False):
    plt.figure(figsize=figure_size)
    # plt.figure()

    base = plt.gca().transData
    rot = transforms.Affine2D().rotate_deg(90)
    #------------------------------------------------------------------------------
    ## set colors for differnt labels
    # Black removed and is used for noise instead.
    unique_labels = set(labels)
    colors = [plt.cm.Spectral(each)
            for each in np.linspace(0, 1, len(unique_labels))]

    point_clusters = []
    cluster = 0 
    while cluster < n_clusters_:
        class_member_mask = (labels == cluster)
        point_clusters.append(point_cloud_numpy[class_member_mask & core_samples_mask])
    #     point_clusters.append(extracted_2D_downsampled[class_member_mask & core_samples_mask])
        cluster = cluster + 1
        
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = (labels == k)
        
        xy = point_cloud_numpy[class_member_mask & core_samples_mask]
    # 
        plt.plot(xy[:, 0], xy[:, 1], '.', markerfacecolor=tuple(col),
                markeredgecolor='None', markersize=14, transform= rot + base)

        # these are the points does not belong to either clusters (noise)
        xy = point_cloud_numpy[class_member_mask & ~core_samples_mask]
    #     xy = extracted_2D_downsampled[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], '.', markerfacecolor=tuple(col),
                markeredgecolor='k', markersize=6, transform= rot + base)
    #-----------------------------------------------------------------------------------------------
    plt.xlabel('X (mm)', fontsize=20, labelpad=10)
    plt.ylabel('Y (mm)', fontsize=20, labelpad=1)
    # plt.xticks(np.arange(-10, 10, 10))
    # plt.xticks([-20, -10, 0 , 10, 20], fontsize = 16) 
    # plt.yticks([-10, 0, 10], fontsize = 16)
    # plt.title('3D Point cloud and reference plane')
    if save:
        save_fig("DBSCAN_visualization_2D")

    return point_clusters



def visualize_fuzzy_c_mean(point_cloud_numpy, u, ncenters, cntr, save = False, show_axis = False):
    x = point_cloud_numpy[:, 0]
    y = point_cloud_numpy[:, 1]
    z = point_cloud_numpy[:, 2]
    # Plot assigned clusters, for each data point in training set
    cluster_membership = np.argmax(u, axis=0)

    plt.figure(figsize=figure_size)
    # plt.figure()
    ax = plt.axes(projection='3d')

    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, ncenters)]


    for j in range(ncenters):
        ax.scatter3D(x[cluster_membership == j], y[cluster_membership == j], z[cluster_membership == j], 
                    c=colors[j], s = 0.5, alpha=0.9)
        # plt.plot(x[cluster_membership == j], y[cluster_membership == j], z[cluster_membership == j] '.', color=colors[j])

    # Mark the center of each fuzzy cluster
    for pt in cntr:
        ax.scatter3D(pt[0], pt[1],pt[2], 'rs')

    # ax.title('Centers = {0}; FPC = {1:.2f}'.format(ncenters, fpc))

    ax.set_xlabel('X (mm)', fontsize=25, labelpad=25)
    ax.set_ylabel('Y (mm)', fontsize=25,labelpad=25)
    ax.set_zlabel('Z (mm)', fontsize=25, labelpad=20);
    # ax.set_zticks([0, 0.5, 1, 1.5, 2])
    # ax.set_zticks([-1, -0.5, 2, 3, 4])
    ax.set_zlim(-1.5, 3)
    ax.set_xticks([-20, -10, 0, 10, 20])
    ax.set_yticks([-30, -20, -10, 0, 10, 20, 30])
    
    if not show_axis:
        ax._axis3don = False

    for t in ax.zaxis.get_major_ticks(): t.label.set_fontsize(20)
    for t in ax.xaxis.get_major_ticks(): t.label.set_fontsize(20)
    for t in ax.yaxis.get_major_ticks(): t.label.set_fontsize(20)
    # plt.title('3D Point cloud and reference plane')
    ax.view_init(elev=60,azim=-20) #rotate the graph
    if save:
        save_fig("Fuzzy_c_mean")


def np_to_shapely(points_np_array):
    '''convert a numpy array of points to shapely points'''
    polygon = geometry.Polygon(np.squeeze(points_np_array))
    line_string = geometry.asLineString(points_np_array)

    return polygon, line_string