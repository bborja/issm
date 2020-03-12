function [data, maxX, minX, maxZ, minZ, p] = samplePCD(ime, n)

%pcdData je sestavljen kot [x, y, z, r, g, b]
pcdData = loadpcd(ime);

%zanimajo nas samo koordinate tock..
%minusi, ker je sicer obrnjeno v pclviewer
pcdTocke1 = [-pcdData(1, :); -pcdData(2, :); pcdData(3, :)];
pcdTocke = pcdTocke1';

%vse tocke pcd datoteke
p = double(pcdTocke);


%naredi randomsample brez ponavljanja iz tock pcd
stTock = length(pcdTocke1(1, :));


pcdSampledTocke = zeros(n, 3);
randomIndices = randperm(stTock);
randomIndices = randomIndices(1:n);

%sample-ane tocke pcd datoteke (za hitrejsi izris tock)
pcdSampledTocke(1:n, 1) = pcdTocke(randomIndices, 1);
pcdSampledTocke(1:n, 2) = pcdTocke(randomIndices, 2);
pcdSampledTocke(1:n, 3) = pcdTocke(randomIndices, 3);
pclviewer(pcdSampledTocke')
figure(1); clf; scatter3(pcdSampledTocke(:,1), pcdSampledTocke(:,2), pcdSampledTocke(:,3));

maxX = max(pcdSampledTocke(:,1));
minX = min(pcdSampledTocke(:,1));

maxZ = max(pcdSampledTocke(:,2));
minZ = min(pcdSampledTocke(:,2));


data = pcdSampledTocke;
    