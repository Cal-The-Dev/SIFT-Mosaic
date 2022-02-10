function mosaic = sift_mosaic(image1, image2, image3, image4, image5)
% Assignment 1 Callum Smith Student ID 200969669 student email
% C.Smith15@student.liv.ac.uk

%CHANGE THE FILE PATH OF IMAGES HERE!
if nargin == 0
  image1 = imread(fullfile('C:\Users\erin9\Desktop\A1_21\data/1.jpg')) ;
  image2 = imread(fullfile('C:\Users\erin9\Desktop\A1_21\data/2.jpg')) ;
  image3 = imread(fullfile('C:\Users\erin9\Desktop\A1_21\data/3.jpg')) ;
  image4 = imread(fullfile('C:\Users\erin9\Desktop\A1_21\data/4.jpg')) ;
  image5 = imread(fullfile('C:\Users\erin9\Desktop\A1_21\data/5.jpg')) ;
end

% make single
image1 = im2single(image1) ;
image2 = im2single(image2) ;
image3 = im2single(image3) ;
image4 = im2single(image4) ;
image5 = im2single(image5) ;


% make grayscale
if size(image1,3) > 1, image1g = rgb2gray(image1) ; else image1g = image1 ; end
if size(image2,3) > 1, image2g = rgb2gray(image2) ; else image2g = image2 ; end
if size(image3,3) > 1, image3g = rgb2gray(image3) ; else image3g = image3 ; end
if size(image4,3) > 1, image4g = rgb2gray(image4) ; else image4g = image4 ; end
if size(image5,3) > 1, image5g = rgb2gray(image5) ; else image5g = image5 ; end


% SIFT MATCHING
[f1,d1] = vl_sift(image1g) ;
[f2,d2] = vl_sift(image2g) ;
[f3,d3] = vl_sift(image3g) ;
[f4,d4] = vl_sift(image4g) ;
[f5,d5] = vl_sift(image5g) ;

%calculate matches for each image pair
[matches, scores] = vl_ubcmatch(d1,d2) ;
[matches2, scores2] = vl_ubcmatch(d2,d3) ;
[matches3, scores3] = vl_ubcmatch(d3,d4) ;
[matches4, scores4] = vl_ubcmatch(d4,d5) ;



X1 = f1(1:2,matches(1,:)) ; X1(3,:) = 1 ;
X2 = f2(1:2,matches(2,:)) ; X2(3,:) = 1 ;

X3 = f2(1:2,matches2(1,:)) ; X3(3,:) = 1;
X4 = f3(1:2,matches2(2,:)) ; X4(3,:) = 1;

X5 = f3(1:2,matches3(1,:)) ; X5(3,:) = 1;
X6 = f4(1:2,matches3(2,:)) ; X6(3,:) = 1;

X7 = f4(1:2,matches4(1,:)) ; X7(3,:) = 1;
X8 = f5(1:2,matches4(2,:)) ; X8(3,:) = 1;

%calculate number of matches to be used later
numMatches = size(matches,2) ;
numMatches2 = size(matches2,2) ;
numMatches3 = size(matches3,2) ;
numMatches4 = size(matches4,2) ;

% RANSAC

clear H score ok ;
clear H2 score2 ok2;
clear H3 score3 ok3;
clear H4 score4 ok4;

for t = 1:100
    
  % Homography
  
  subset = vl_colsubset(1:numMatches, 4) ;
  subset2 = vl_colsubset(1:numMatches2, 4) ;
  subset3 = vl_colsubset(1:numMatches3, 4) ;
  subset4 = vl_colsubset(1:numMatches4, 4) ;
  
  A = [] ;
  B = [] ;
  C = [] ;
  D = [] ;
  
  for i = subset
    A = cat(1, A, kron(X1(:,i)', vl_hat(X2(:,i)))) ;
  end
  
  for i = subset2
    B = cat(1, B, kron(X3(:,i)', vl_hat(X4(:,i)))) ;
  end
  
  for i = subset3
    C = cat(1, C, kron(X5(:,i)', vl_hat(X6(:,i)))) ;
  end
  
  for i = subset4
    D = cat(1, D, kron(X7(:,i)', vl_hat(X8(:,i)))) ;
  end
      
    
  
  [U,S,V] = svd(A) ;
  H{t} = reshape(V(:,9),3,3) ;
  
    
  [U2,S2,V2] = svd(B) ;
  H2{t} = reshape(V2(:,9),3,3) ;
  
    
  [U3,S3,V3] = svd(C) ;
  H3{t} = reshape(V3(:,9),3,3) ;
  
    
  [U4,S4,V4] = svd(D) ;
  H4{t} = reshape(V4(:,9),3,3) ;

  % Scoring
  
  X2_ = H{t} * X1 ;
  du = X2_(1,:)./X2_(3,:) - X2(1,:)./X2(3,:) ;
  dv = X2_(2,:)./X2_(3,:) - X2(2,:)./X2(3,:) ;
  ok{t} = (du.*du + dv.*dv) < 6*6 ;
  score(t) = sum(ok{t}) ;
  
  X4_ = H2{t} * X3 ;
  du2 = X4_(1,:)./X4_(3,:) - X4(1,:)./X4(3,:) ;
  dv2 = X4_(2,:)./X4_(3,:) - X4(2,:)./X4(3,:) ;
  ok2{t} = (du2.*du2 + dv2.*dv2) < 6*6 ;
  score2(t) = sum(ok2{t}) ;
  
  X6_ = H3{t} * X5 ;
  du3 = X6_(1,:)./X6_(3,:) - X6(1,:)./X6(3,:) ;
  dv3 = X6_(2,:)./X6_(3,:) - X6(2,:)./X6(3,:) ;
  ok3{t} = (du3.*du3 + dv3.*dv3) < 6*6 ;
  score3(t) = sum(ok3{t}) ;
  
  X8_ = H4{t} * X7 ;
  du4 = X8_(1,:)./X8_(3,:) - X8(1,:)./X8(3,:) ;
  dv4 = X8_(2,:)./X8_(3,:) - X8(2,:)./X8(3,:) ;
  ok4{t} = (du4.*du4 + dv4.*dv4) < 6*6 ;
  score4(t) = sum(ok4{t}) ;
  
  
end

[score, best] = max(score) ;
H = H{best} ;
ok = ok{best} ;

[score2, best2] = max(score2) ;
H2 = H2{best2} ;
ok2 = ok2{best2} ;

[score3, best3] = max(score3) ;
H3 = H3{best3} ;
ok3 = ok3{best3} ;

[score4, best4] = max(score4) ;
H4 = H4{best4} ;
ok4 = ok4{best4} ;

% Show Matches -------------------------------------------

dh1 = max(size(image2,1)-size(image1,1),0) ;
dh2 = max(size(image1,1)-size(image2,1),0) ;

figure(1) ; clf ;
subplot(2,1,1) ;
imagesc([padarray(image1,dh1,'post') padarray(image2,dh2,'post')]) ;
o = size(image1,2) ;
line([f1(1,matches(1,:));f2(1,matches(2,:))+o], ...
     [f1(2,matches(1,:));f2(2,matches(2,:))]) ;
title(sprintf('%d tentative matches for image 1 & 2', numMatches)) ;
axis image off ;

subplot(2,1,2) ;
imagesc([padarray(image1,dh1,'post') padarray(image2,dh2,'post')]) ;
o = size(image1,2) ;
line([f1(1,matches(1,ok));f2(1,matches(2,ok))+o], ...
     [f1(2,matches(1,ok));f2(2,matches(2,ok))]) ;
title(sprintf('%d (%.2f%%) inliner matches out of %d', ...
              sum(ok), ...
              100*sum(ok)/numMatches, ...
              numMatches)) ;
axis image off ;

drawnow ;
%
%
%
dh3 = max(size(image3,1)-size(image2,1),0) ;
dh4 = max(size(image2,1)-size(image3,1),0) ;

figure(2) ; clf ;
subplot(2,1,1) ;
imagesc([padarray(image2,dh3,'post') padarray(image3,dh4,'post')]) ;
o2 = size(image2,2) ;
line([f2(1,matches2(1,:));f3(1,matches2(2,:))+o2], ...
     [f2(2,matches2(1,:));f3(2,matches2(2,:))]) ;
title(sprintf('%d tentative matches for image 2 & 3', numMatches2)) ;
axis image off ;

subplot(2,1,2) ;
imagesc([padarray(image2,dh3,'post') padarray(image3,dh4,'post')]) ;
o2 = size(image2,2) ;
line([f2(1,matches2(1,ok2));f3(1,matches2(2,ok2))+o2], ...
     [f2(2,matches2(1,ok2));f3(2,matches2(2,ok2))]) ;
title(sprintf('%d (%.2f%%) inliner matches out of %d', ...
              sum(ok2), ...
              100*sum(ok2)/numMatches2, ...
              numMatches2)) ;
axis image off ;

drawnow ;

%
%
%
dh5 = max(size(image4,1)-size(image3,1),0) ;
dh6 = max(size(image3,1)-size(image4,1),0) ;

figure(3) ; clf ;
subplot(2,1,1) ;
imagesc([padarray(image3,dh5,'post') padarray(image4,dh6,'post')]) ;
o3 = size(image3,2) ;
line([f3(1,matches3(1,:));f4(1,matches3(2,:))+o3], ...
     [f3(2,matches3(1,:));f4(2,matches3(2,:))]) ;
title(sprintf('%d tentative matches for image 3 & 4', numMatches3)) ;
axis image off ;

subplot(2,1,2) ;
imagesc([padarray(image3,dh5,'post') padarray(image4,dh6,'post')]) ;
o3 = size(image3,2) ;
line([f3(1,matches3(1,ok3));f4(1,matches3(2,ok3))+o3], ...
     [f3(2,matches3(1,ok3));f4(2,matches3(2,ok3))]) ;
title(sprintf('%d (%.2f%%) inliner matches out of %d', ...
              sum(ok3), ...
              100*sum(ok3)/numMatches3, ...
              numMatches3)) ;
axis image off ;

drawnow ;

%
%
%
dh7 = max(size(image5,1)-size(image4,1),0) ;
dh8 = max(size(image4,1)-size(image5,1),0) ;

figure(4) ; clf ;
subplot(2,1,1) ;
imagesc([padarray(image4,dh7,'post') padarray(image5,dh8,'post')]) ;
o4 = size(image4,2) ;
line([f4(1,matches4(1,:));f5(1,matches4(2,:))+o4], ...
     [f4(2,matches4(1,:));f5(2,matches4(2,:))]) ;
title(sprintf('%d tentative matches for image 4 & 5', numMatches4)) ;
axis image off ;

subplot(2,1,2) ;
imagesc([padarray(image4,dh7,'post') padarray(image5,dh8,'post')]) ;
o4 = size(image4,2) ;
line([f4(1,matches4(1,ok4));f5(1,matches4(2,ok4))+o4], ...
     [f4(2,matches4(1,ok4));f5(2,matches4(2,ok4))]) ;
title(sprintf('%d (%.2f%%) inliner matches out of %d', ...
              sum(ok4), ...
              100*sum(ok4)/numMatches4, ...
              numMatches4)) ;
axis image off ;

drawnow ;


% --------------------------------------------------------------------
%                                                               Mosaic
% --------------------------------------------------------------------

%Create second image transformation as a "Box"
box2 = [1  size(image2,2) size(image2,2)  1 ;
        1  1           size(image2,1)  size(image2,1) ;
        1  1           1            1 ] ;    
box2_ = inv(H) * box2 ;
box2_(1,:) = box2_(1,:) ./ box2_(3,:) ;
box2_(2,:) = box2_(2,:) ./ box2_(3,:) ;

%Vice versa for 3
box3 = [1  size(image3,2) size(image3,2)  1 ;
        1  1           size(image3,1)  size(image3,1) ;
        1  1           1            1 ] ;    
box3_ = inv(H2) * box3 ;
box3_(1,:) = box3_(1,:) ./ box3_(3,:) ;
box3_(2,:) = box3_(2,:) ./ box3_(3,:) ;

%Vice versa for 4
box4 = [1  size(image4,2) size(image4,2)  1 ;
        1  1           size(image4,1)  size(image4,1) ;
        1  1           1            1 ] ;    
box4_ = inv(H3) * box4 ;
box4_(1,:) = box4_(1,:) ./ box4_(3,:) ;
box4_(2,:) = box4_(2,:) ./ box4_(3,:) ;

%Vice versa for 5
box5 = [1  size(image5,2) size(image5,2)  1 ;
        1  1           size(image5,1)  size(image5,1) ;
        1  1           1            1 ] ;    
box5_ = inv(H4) * box5 ;
box5_(1,:) = box5_(1,:) ./ box5_(3,:) ;
box5_(2,:) = box5_(2,:) ./ box5_(3,:) ;


%CALCULATE SIZE OF STITCHED IMAGE D:
ur = min([1 box2_(1,:) box3_(1,:) box4_(1,:) box5_(1,:)]):max([size(image1,2) box2_(1,:) box3_(1,:) box4_(1,:) box5_(1,:)]) ;
vr = min([1 box2_(1,:) box3_(1,:) box4_(1,:) box5_(1,:)]):max([size(image1,1) box2_(2,:) box3_(1,:) box4_(1,:) box5_(1,:)]) ;

[u,v] = meshgrid((ur * 2),(vr * 2)) ;
image1_ = vl_imwbackward(im2double(image1),u,v) ;

%Calculating final transformations
z_ = H(3,1) * u + H(3,2) * v + H(3,3) ;
u_ = (H(1,1) * u + H(1,2) * v + H(1,3)) ./ z_ ;
v_ = (H(2,1) * u + H(2,2) * v + H(2,3)) ./ z_ ;
image2_ = vl_imwbackward(im2double(image2),u_,v_) ;

z2_ = H2(3,1) * u_ + H2(3,2) * v_ + H2(3,3) ;
u2_ = (H2(1,1) * u_ + H2(1,2) * v_ + H2(1,3)) ./ z2_ ;
v2_ = (H2(2,1) * u_ + H2(2,2) * v_ + H2(2,3)) ./ z2_ ;
image3_ = vl_imwbackward(im2double(image3),u2_,v2_) ;

z3_ = H3(3,1) * u2_ + H3(3,2) * v2_ + H3(3,3) ;
u3_ = (H3(1,1) * u2_ + H3(1,2) * v2_ + H3(1,3)) ./ z3_ ;
v3_ = (H3(2,1) * u2_ + H3(2,2) * v2_ + H3(2,3)) ./ z3_ ;
image4_ = vl_imwbackward(im2double(image4),u3_,v3_) ;

z4_ = H4(3,1) * u3_ + H4(3,2) * v3_ + H4(3,3) ;
u4_ = (H4(1,1) * u3_ + H4(1,2) * v3_ + H4(1,3)) ./ z4_ ;
v4_ = (H4(2,1) * u3_ + H4(2,2) * v3_ + H4(2,3)) ./ z4_ ;
image5_ = vl_imwbackward(im2double(image5),u4_,v4_) ;

mass = ~isnan(image1_) + ~isnan(image2_) + ~isnan(image4_) + ~isnan(image4_) + ~isnan(image5_) ;
image1_(isnan(image1_)) = 0 ;
image2_(isnan(image2_)) = 0 ;

image3_(isnan(image3_)) = 0 ;
image4_(isnan(image4_)) = 0 ;
image5_(isnan(image5_)) = 0 ;

%Finalising Mosaic
mosaic = (image1_ + image2_+ image3_+ image4_+ image5_) ./ mass ;

figure(5) ; clf ;
imagesc(mosaic) ; axis image off ;
title('Mosaic') ;

if nargout == 0, clear mosaic ; end

%Writing final image
saveas(figure(5), 'Stitched.jpg')

end