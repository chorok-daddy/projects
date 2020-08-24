import os

os.system("ffmpeg -r 15 -i ./figs/fig%d.png -vcodec mpeg4 -y movie.mp4")
#os.system("rm ./figs/*")