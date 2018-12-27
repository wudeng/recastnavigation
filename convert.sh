#!/bin/bash

#set -ex

Convertor="./RecastDemo/Bin/Convertor"
ClientDir="/work/omclient/Assets/Arts/scene"

ClientMeshes=$(find $ClientDir -name NavMesh.asset)

mkdir -p navmesh

if true; then
for ClientMesh in $ClientMeshes; do
    echo $ClientMesh
    DirName=$(dirname $ClientMesh)
    ServerMesh="navmesh/${DirName##*/}.bin"
    $Convertor $ClientMesh $ServerMesh
done
fi

clientfile=./RecastDemo/Build/vs2015/tokyo_off.asset
serverfile=./RecastDemo/Build/vs2015/tokyo_off.bin
$Convertor $clientfile $serverfile

clientfile=tokyo_10j_5d.asset
serverfile=tokyo_10j_5d.bin
$Convertor $clientfile $serverfile
