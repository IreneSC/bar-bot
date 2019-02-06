function [centroidx, centroidy, isFaceFirstDetected, isFaceDetected, numPts, oldPoints, bboxPoints] ...
= faceTrack(faceDetector, pointTracker, cam, videoPlayer, numPtsOG, oldPointsOG, bboxPointsOG)
%% Dectection + Tracking
numPts = numPtsOG;
oldPoints = oldPointsOG;
bboxPoints = bboxPointsOG;
isFaceFirstDetected = 0;
isFaceDetected = 0;
centroidx = 0;
centroidy = 0;

min_points_for_face = 50;

% Get the next frame.
videoFrame = snapshot(cam);
videoFrameGray = rgb2gray(videoFrame);

if numPts < min_points_for_face
    % Detection mode.
    bbox = faceDetector.step(videoFrameGray);
    if ~isempty(bbox)
        isFaceFirstDetected = 1;
        % Find corner points inside the detected region.
        points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

        % Re-initialize the point tracker.
        xyPoints = points.Location;
        numPts = size(xyPoints,1);
        release(pointTracker);
        initialize(pointTracker, xyPoints, videoFrameGray);

        % Save a copy of the points.
        oldPoints = xyPoints;

        % Convert the rectangle represented as [x, y, w, h] into an
        % M-by-2 matrix of [x,y] coordinates of the four corners. This
        % is needed to be able to transform the bounding box to display
        % the orientation of the face.
        bboxPoints = bbox2points(bbox(1, :));

        % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
        % format required by insertShape.
        bboxPolygon = reshape(bboxPoints', 1, []);

        % Display a bounding box around the detected face.
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

        % Display detected corners.
        videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
    end

else
    % Tracking mode.
    [xyPoints, isFound] = step(pointTracker, videoFrameGray);
    visiblePoints = xyPoints(isFound, :);
    oldInliers = oldPoints(isFound, :);

    numPts = size(visiblePoints, 1);

    if numPts >= min_points_for_face
        isFaceDetected = 1;
        fprintf("found face with %d points\n", numPts);
        % Estimate the geometric transformation between the old points
        % and the new points.
        [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
            oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

        % Apply the transformation to the bounding box.
        bboxPoints = transformPointsForward(xform, bboxPoints);

        % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
        % format required by insertShape.
        bboxPolygon = reshape(bboxPoints', 1, []);

        % Display a bounding box around the face being tracked.
        videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

        % Display tracked points.
        videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

        % Reset the points.
        oldPoints = visiblePoints;
        setPoints(pointTracker, oldPoints);
    end

end

% Display the annotated video frame using the video player object.
step(videoPlayer, videoFrame);

if isFaceDetected
    centroidx = mean(bboxPoints(:,1));
    centroidy = mean(bboxPoints(:,2));
    fprintf("cx: %f, cy: %f\n", centroidx, centroidy);
end

end

