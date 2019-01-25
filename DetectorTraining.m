positiveInstances = label_table(:,1:2);
imDir = fullfile('Box Detector');
addpath(imDir);
negativeFolder = fullfile('Box Detector', 'No Arm');
negativeImages = imageDatastore(negativeFolder);
trainCascadeObjectDetector('armDetector.xml',positiveInstances, ...
    negativeFolder,'FalseAlarmRate',0.02,'NumCascadeStages',5);