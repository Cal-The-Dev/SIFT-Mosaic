Introduction
Our aim is to take a minimum of 5 images and use Vlfeat and Matlab to detect
key points in the images, build SIFT descriptors and match the images together, transforming
them together to create a combined panoramic image using RANSAC. Finally we will output that
image to an image file on the computer.

How we  achieve this.

We aree using VLfeats tutorial code for dual image panoramic images and both
learning and modifying it to accept and stitch 5 images together, displaying the SIFT
descriptors and matches for each image.

Vl feats code gives a standard template for SIFT panorama stitching, creating a
“Mosaic” out of 2 images. By creating our own custom code we will take in a further 3
images as input and sequentially compare side by side images in order to finally
transform and stitch them together.

Using Matlabs plotting tools we will then display SIFT matches between images, then it
will show below that the matches with Inliers removed and the % of matches left.
In order to achieve this we will create new objects, known as “Boxes” which are a
transformed image, created to overlay and match up with each previous image.
Stitching the original image and 4 of these boxes together sequentially will create our
final panorama image.

We will then simply use Matlab’s inbuilt functions to output that image to a file. This
should work for any 5 images put in the input folder, which holds images to be
processed.

Improvements
In order to aid with optimisation, unnecessary components of the tutorial code
will be ignored when we create our custom version, stripping the problem down to its
bare components and replicating them for each of our image pairs.
We will also remove code related to if there’s no input arguments, as with 5 images
there will always be input so this is unnecessary.

How to run the code

To run the code, load up the SIFT_Mosaic.m file in Matlab.
Vl Feat must be downloaded, and you must activate VL Feat upon running matlab
through the command “run('VL ROOT DIRECTORY/VLFEATROOT/toolbox/vl_setup')
replacing VL ROOT DIRECTORY with the location you’ve downloaded it to.

Go to line 7 to 11 in SIFT_Mosaic.m and replace the image directory for each image,
with the directory you have your 5 images within.

The program will show matched image SIFT features between the pictures and finally
the completed mosaic, which will then be saved as an outputted image file.
The output image file will be saved in whatever directory SIFT_Mosaic.m is run.