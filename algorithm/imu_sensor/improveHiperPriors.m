function [hiperPriorsMus, landHiperPriorCov] = improveHiperPriors(hiperPriorsMus, landHiperPriorCov, centerHorY, avgHorizonCenterY, phi, size_img)   
    avgHorizonCenterY = avgHorizonCenterY.avgHorizonCenterYnormalized;
	
	%learned_hip_height = 50;
    learned_hip_height = 100;
    
    lambda1 = avgHorizonCenterY - hiperPriorsMus(1,2) / learned_hip_height(1);  %sky
    lambda2 = avgHorizonCenterY - hiperPriorsMus(2,2) / learned_hip_height(1);  %land
    lambda3 = hiperPriorsMus(3,2) / learned_hip_height(1) - avgHorizonCenterY;  %sea
    
    hiperPriorsMus(1,2) = (centerHorY - lambda1) * size_img(1);
    hiperPriorsMus(2,2) = (centerHorY - lambda2) * size_img(1);
    hiperPriorsMus(3,2) = (centerHorY + lambda3) * size_img(1);
    
    landHiperPriorCov(1:2, 1:2) = modify_cov_matrix(landHiperPriorCov(1:2, 1:2), phi);
end