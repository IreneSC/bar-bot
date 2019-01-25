function [centroidx, centroidy, isFaceDetected, numPts] = faceTrack(faceDetector, pointTracker, cam, videoPlayer, numPtsOG)
%% Dectection + Tracking
numPts = numPtsOG;
isFaceDetected = 0;
centroidx = 0;
centroidy = 0;

% Get the next frame.
videoFrame = snapshot(cam);
videoFrameGray = rgb2gray(videoFrame);

if numPts < 30
    % Detection mode.
    bbox = faceDetector.step(videoFrameGray);
    if ~isempty(bbox)
        isFaceDetected = 1;
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
    isFaceDetected = isFound;
    visiblePoints = xyPoints(isFound, :);
    oldInliers = oldPoints(isFound, :);

    numPts = size(visiblePoints, 1);

    if numPts >= 30
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
    numPts = numPts + 1;
else
    numPts = 0;
end

end

