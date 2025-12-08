#!/usr/bin/env python3
import cv2
import sys
import glob
import os
import argparse

p = argparse.ArgumentParser(description='Check chessboard detection on images or folder')
p.add_argument('input', help='Image path or glob (e.g. ~/images/*.jpg) or single file')
p.add_argument('--board', default='8x5', help='inner corners WxH (default 8x5)')
p.add_argument('--outdir', default='/tmp/check_results', help='where to write debug images')
args = p.parse_args()

w,h = map(int, args.board.split('x'))
imgs = []
if os.path.isdir(args.input):
    imgs = sorted(glob.glob(os.path.join(args.input, '*'))) 
else:
    imgs = sorted(glob.glob(os.path.expanduser(args.input)))

if not imgs:
    print('No images found for', args.input)
    sys.exit(1)

os.makedirs(args.outdir, exist_ok=True)

for f in imgs:
    img = cv2.imread(f)
    if img is None:
        print('failed to load', f)
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (w,h), flags)
    print(f, 'Found:', found)
    if found:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), term)
        cv2.drawChessboardCorners(img, (w,h), corners2, found)
        out = os.path.join(args.outdir, os.path.basename(f))
        cv2.imwrite(out, img)
        print('Wrote debug image to', out)

print('Done. Debug images (if any) are in', args.outdir)
