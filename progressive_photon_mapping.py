import os
import numpy

import PIL
from PIL import Image
if __name__ == "__main__":
    lista = [10,50,100,200]
    for i in numpy.arange(len(lista)):
        command = "./bin/yscene render"
        out_file = "imagen_"+str(i)
        photons = str(lista[i])
        samples = str(32)

        scene = "iluminacion_vertical"

        command = command + " --sampler path --photon_mapping --samples "+samples
        command = command + " --output ./imagenes/"+out_file+'.png'
        command = command + " ./tests/"+scene+"/"+scene+".json"
        #print(command)
        os.system(command)
    

    # Access all PNG files in directory
    allfiles=os.listdir(os.getcwd())
    imlist=[filename for filename in allfiles if  filename[-4:] in [".png",".PNG"]]

    # Assuming all images are the same size, get dimensions of first image
    w,h=Image.open(imlist[0]).size
    N=len(imlist)

    # Create a numpy array of floats to store the average (assume RGB images)
    imarr_shape=numpy.array(Image.open(imlist[0]),dtype=numpy.float).shape
    arr=numpy.zeros(imarr_shape,numpy.float)

    # Build up average pixel intensities, casting each image as an array of floats
    for im in imlist:
        imarr=numpy.array(Image.open(im),dtype=numpy.float)
        arr=arr+imarr/N

    # Round values in array and cast as 8-bit integer
    arr=numpy.array(numpy.round(arr),dtype=numpy.uint8)

    # Generate, save and preview final image
    out=Image.fromarray(arr,mode="RGBA")
    out.save("Average.png")
    out.show()