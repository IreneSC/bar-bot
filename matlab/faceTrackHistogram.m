function [centroidx, centroidy, isFaceFirstDetected, isFaceDetected, numPts, oldPoints, bboxPoints, histogramTracker] ...
= faceTrackHistogram(faceDetector, histogramTrackerOG, cam, videoPlayer, numPtsOG, oldPointsOG, bboxPointsOG)
%% Dectection + Tracking
numPts = numPtsOG;
oldPoints = oldPointsOG;
bboxPoints = bboxPointsOG;
histogramTracker = histogramTrackerOG;
isFaceFirstDetected = 0;
isFaceDetected = 0;
centroidx = 0;
centroidy = 0;

min_points_for_face = 50;

% Get the next frame.
videoFrame = snapshot(cam);
videoFrameGray = rgb2gray(videoFrame);
videoFrameHSV = rgb2hsv(videoFrame);

if ~numPts
    % Detection mode.
    bbox = faceDetector.step(videoFrameGray);
    if ~isempty(bbox)
        fprintf("saw it!\n");
        isFaceFirstDetected = 1;

        % Convert the rectangle represented as [x, y, w, h] into an
        % M-by-2 matrix of [x,y] coordinates of the four corners. This
        % is needed to be able to transform the bounding box to display
        % the orientation of the face.
        %bboxPoints = bbox2points(bbox(1, :));

        % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
        % format required by insertShape.
        %bboxPolygon = reshape(bboxPoints', 1, []);
        disp(bbox);
        initializeObject(histogramTracker, videoFrameHSV(:,:,1), bbox(1,:));

        % Display a bounding box around the detected face.
        videoFrame = insertShape(videoFrame, 'Rectangle', bbox, 'Color', 'yellow');

        % Display detected corners.
        %videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        numPts = 1;
    end

else
    % Tracking mode.
    [bbox, orient, score] = step(histogramTracker, videoFrameHSV(:,:,1));

    if ~isempty(bbox) && score > 0.3
        isFaceDetected = 1;
        % Estimate the geometric transformation between the old points
        % and the new points.
        %[xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
        %    oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

        % Apply the transformation to the bounding box.
        %bboxPoints = transformPointsForward(xform, bboxPoints);

        % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
        % format required by insertShape.
        %bboxPolygon = reshape(bboxPoints', 1, []);

        % Display a bounding box around the face being tracked.
        videoFrame = insertShape(videoFrame, 'Rectangle', bbox, 'Color', 'yellow');

        % Display tracked points.
        %videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

        % Reset the points.
        %oldPoints = visiblePoints;
        %setPoints(histogramTracker, oldPoints);
    else
        numPts = 0;
    end

end

% Display the annotated video frame using the video player object.
step(videoPlayer, videoFrame);

if isFaceDetected
    centroidx = bbox(1) + bbox(3)/2;
    centroidy = bbox(2) + bbox(4)/2;
    %fprintf("cx: %f, cy: %f\n", centroidx, centroidy);
end

end

