orthophoto = imread('BaseBall Fixed.png');
figure, imshow(orthophoto)
original = imread('BaseBall SatView 2.png');
figure, imshow(original)
cpselect(original, orthophoto)
tform = estimateGeometricTransform(input_points,base_points,'projective');

outputView = imref2d(size(orthophoto));
Ir = imwarp(original, tform, 'OutputView', outputView);
figure; imshow(Ir); title('Recovered image');
