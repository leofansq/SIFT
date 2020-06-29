from PIL import Image
import os.path
import glob
from argparse import ArgumentParser
 
def img2pgm(img_file, save_path):
    img = Image.open(img_file).convert('L')
    save_name =(str)(os.path.join(save_path, os.path.splitext(os.path.basename(img_file))[0]))+".pgm"
    img.save(save_name)
    print ("Done")
 

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-ImageType', default="png", help="png or jpg")
    parser.add_argument('-SavePath', default="./", help="save path for pgm files")

    arg = parser.parse_args()

    img_type = arg.ImageType
    save_path = arg.SavePath

    cnt = 0
    for img_file in glob.glob("./*.{}".format(img_type)):
        cnt += 1
        print ("{} --- Load  Converting...".format(img_file), end="  ")
        img2pgm(img_file, save_path)
    
    if cnt == 0: print ("Could not read any image. Please confirm the IMAGE TYPE!")
    else: print ("{} file(s) have converted to PGM format. Saved at {}".format(cnt, save_path))
