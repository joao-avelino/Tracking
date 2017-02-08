function [ K, RT ] = readCameraParameters( filePath )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

xmlDoc = xmlread(filePath);

camera = xmlDoc.getDocumentElement();
children = camera.getChildNodes();

for i=0:children.getLength-1

    node = children.item(i);
    nodename = node.getNodeName;
    
    if strcmp(nodename, 'Geometry')
       mDpx = str2double(node.getAttribute('dpx'));
       mDpy = str2double(node.getAttribute('dpy'));
    end
    
    if strcmp(nodename, 'Intrinsic')
       mFocal = str2double(node.getAttribute('focal'));
       mSx = str2double(node.getAttribute('sx'));
       mCx = str2double(node.getAttribute('cx'));
       mCy = str2double(node.getAttribute('cy'));
    end
    
    if strcmp(nodename, 'Extrinsic')
       sa = sin(str2double(node.getAttribute('rx')));
       ca = cos(str2double(node.getAttribute('rx')));
       sb = sin(str2double(node.getAttribute('ry')));
       cb = cos(str2double(node.getAttribute('ry')));
       sg = sin(str2double(node.getAttribute('rz')));
       cg = cos(str2double(node.getAttribute('rz')));
       mTx = str2double(node.getAttribute('tx'));
       mTy = str2double(node.getAttribute('ty'));
       mTz = str2double(node.getAttribute('tz'));
    end
end

%Now that the parsing is complete compute the extrinsic matrix

mR11 = cb * cg;
mR12 = cg * sa * sb - ca * sg;
mR13 = sa * sg + ca * cg * sb;
mR21 = cb * sg;
mR22 = sa * sb * sg + ca * cg;
mR23 = ca * sb * sg - cg * sa;
mR31 = -sb;
mR32 = cb * sa;
mR33 = ca * cb;

RT = [mR11 mR12 mR13 mTx;
    mR21 mR22 mR23 mTy;
    mR31 mR32 mR33 mTz];

K = [mFocal*mSx/mDpx 0 mCx;
    0 mFocal/mDpy mCy;
    0 0 1];
