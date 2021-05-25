import os
import numpy as np
if __name__ == "__main__":
    lista = [10,50,100,200,500]
    for i in np.arange(len(lista)):
        command = "./bin/yscene render"
        out_file = "imagen_"+str(i)
        photons = str(lista[i])
        samples = str(16)

        scene = "iluminacion_vertical"

        command = command + " --sampler path --photon_mapping --samples "+samples
        command = command + " --output ./imagenes/"+out_file
        command = command + " ./tests/"+scene+"/"+scene+".json"
        #print(command)
        os.system(command)