docker run -dit \
--privileged \
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--network=host -e DISPLAY=$DISPLAY \
-v /home/jeewonkim/sgraph_ws/workspaces:/root/workspaces \
--name s_graphs_container s_graphs


#docker run -it -t \
#-e DISPLAY=$DISPLAY \
#-v /tmp/.X11-unix:/tmp/.X11-unix \
#--env="QT_X11_NO_MITSHM=1" \
#-v /home/jeewon/sgraph_ws/workspaces:/root/workspaces \
#--name s_graphs_container s-graph

