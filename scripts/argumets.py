#!/usr/bin/env python

import sys
import argparse



def main(model = 'mesa',path = 'teste',pose = [0,0,0]):
    print(model,path)
    pass


if __name__ =='__main__':
    
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="model name")
    argParser.add_argument("-mp", "--modelpath", help="model path")
    args = argParser.parse_args()

    main(model = args.name,path = args.modelpath)


