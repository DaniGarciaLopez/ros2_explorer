#!/usr/bin/env python

# Authors: Juan G Victores
# CopyPolicy: released under the terms of the LGPLv2.1
# URL: https://github.com/roboticslab-uc3m/gazebo-tools

# Modified by: Daniel Garcia Lopez
# URL: https://github.com/DaniGarciaLopez/ros2_explorer

#Modified from original to load all walls as a fixed model instead of loading every square as a unique model. Better performance.

from lxml import etree
import os

#Get all names from csv files in 'maps' folder
path_to_csv = 'maps/'
filenames = [f for f in os.listdir(path_to_csv) if f.endswith('.csv')] #Get all filenames from folder ending in .csv

#Raise error if folder is empty
if not filenames:
    raise ValueError('No CSV files in maps folder')

for filename in filenames: #Iterate through all csv filenames
    filename=filename[:-4] #Delete .csv extension
    #-- User variables
    boxHeight = 1.0
    inFileStr = 'maps/'+filename+'.csv'

    resolution = 1.0  # Just to make similar to MATLAB [pixel/meter]
    meterPerPixel = 1 / resolution  # [meter/pixel]

    #-- Program
    from numpy import genfromtxt
    inFile = genfromtxt(inFileStr, delimiter=',')
    print(inFile)

    nX = inFile.shape[0]
    nY = inFile.shape[1]
    print("lines = X =",inFile.shape[0])
    print("columns = Y =",inFile.shape[1])

    #-- Default to X=rows,Y=columns. Uncomment the next 3 lines to transpose.
    # print("transposing")
    # from numpy import transpose
    # inFile = transpose(inFile)

    Ez = boxHeight

    Ex = meterPerPixel
    Ey = meterPerPixel

    sdf = etree.Element("sdf", version="1.5")
    # world = etree.SubElement(sdf, "world", name="default")
    # light = etree.SubElement(world, "light", name="sun", type="directional")
    # cast_shadows = etree.SubElement(light, "cast_shadows").text="1"
    # diffuse = etree.SubElement(light, "diffuse").text="0.8 0.8 0.8 1"
    # specular = etree.SubElement(light, "specular").text="0.1 0.1 0.1 1"
    # attenuation = etree.SubElement(light, "attenuation")
    # _range = etree.SubElement(attenuation, "range").text="1000"
    # constant = etree.SubElement(attenuation, "constant").text="0.9"
    # linear = etree.SubElement(attenuation, "linear").text="0.01"
    # quadratic = etree.SubElement(attenuation, "quadratic").text="0.001"
    # direction = etree.SubElement(light, "direction").text="-0.5 0.5 -1"

    #-- Create Floor
    floorEx = Ex * nX
    floorEy = Ey * nY
    floorEz = boxHeight / 8.0  # arbitrary

    model = etree.SubElement(sdf, "model", name="map")
    static = etree.SubElement(model, "static").text="1"
    link = etree.SubElement(model, "link", name="map")
    collision = etree.SubElement(link, "collision", name="floor")
    pose = etree.SubElement(collision, "pose").text=str(floorEx/2.0)+" "+str(floorEy/2.0)+" "+str(-floorEz/2.0)+" 0 0 0"
    geometry = etree.SubElement(collision, "geometry")
    box = etree.SubElement(geometry, "box")
    size = etree.SubElement(box, "size").text=str(floorEx)+" "+ str(floorEy)+" "+str(floorEz)
    visual = etree.SubElement(link, "visual", name="floor")
    pose = etree.SubElement(visual, "pose").text=str(floorEx/2.0)+" "+str(floorEy/2.0)+" "+str(-floorEz/2.0)+" 0 0 0"
    geometry = etree.SubElement(visual, "geometry")
    box = etree.SubElement(geometry, "box")
    size = etree.SubElement(box, "size").text=str(floorEx)+" "+ str(floorEy)+" "+str(floorEz)

    #-- Create Walls
    for iX in range(nX):
        #print("iX:",iX)
        for iY in range(nY):
            #print("* iY:",iY)

            #-- Skip box if map indicates a 0
            if inFile[iX][iY] == 0:
                continue

            #-- Add E___/2.0 to each to force begin at 0,0,0 (centered by default)
            x = Ex/2.0 + iX*meterPerPixel
            y = Ey/2.0 + iY*meterPerPixel
            z = Ez/2.0  # Add this to raise to floor level (centered by default)

            #-- Create box
            name="box_"+str(iX)+"_"+str(iY)
            #model = etree.SubElement(world, "model", name="box_"+str(iX)+"_"+str(iY))
            #static = etree.SubElement(model, "static").text="true"
            #link = etree.SubElement(model, "link", name="box_"+str(iX)+"_"+str(iY))
            collision = etree.SubElement(link, "collision", name=name)
            pose = etree.SubElement(collision, "pose").text=str(x)+" "+str(y)+" "+str(z)+" 0 0 0"
            geometry = etree.SubElement(collision, "geometry")
            box = etree.SubElement(geometry, "box")
            size = etree.SubElement(box, "size").text=str(Ex)+" "+ str(Ey)+" "+str(Ez)
            
            visual = etree.SubElement(link, "visual", name=name)
            pose = etree.SubElement(visual, "pose").text=str(x)+" "+str(y)+" "+str(z)+" 0 0 0"
            geometry = etree.SubElement(visual, "geometry")
            box = etree.SubElement(geometry, "box")
            size = etree.SubElement(box, "size").text=str(Ex)+" "+ str(Ey)+" "+str(Ez)

    myStr = etree.tostring(sdf, pretty_print=True, encoding="unicode")

    #Create map folder
    path=os.path.join('models', filename)
    if not os.path.exists(path):
        os.makedirs(path)
    path=os.path.join('models', filename)

    #Write .sdf file
    outFile = open(os.path.join(path, filename+'.sdf'), 'w')
    outFile.write(myStr)
    outFile.close()

    #Create model.config file
    model = etree.Element("model")
    name = etree.SubElement(model, "name").text='CSV Map'
    version = etree.SubElement(model, "version").text='1.0'
    sdf = etree.SubElement(model, "sdf", version='1.5').text=filename+'.sdf'

    # author = etree.SubElement(model, "author")
    # name = etree.SubElement(author, "name").text=''
    # name = etree.SubElement(author, "email").text=''

    description = etree.SubElement(model, "description").text='Map generated automatically from a csv file. More info: https://github.com/DaniGarciaLopez/ros2_explorer'

    #Write model.config file
    myStr = etree.tostring(model, pretty_print=True, encoding="unicode")
    outFile = open(os.path.join(path, 'model.config'), 'w')
    outFile.write(myStr)
    outFile.close()


