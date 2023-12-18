if [ $# -eq 0 ]; then
	echo 'usage '$0' <new_id>'
	exit
fi
sudo sed -i 's/ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID='$1'/g' /etc/environment
while true; do
	read -p 'You need to reboot to apply change. Do you want reboot now ? [y/N]' yn
	case  in
		[Yy]* ) sudo reboot; break;;
		[Nn]* ) exit;;
		* ) exit;;
 esac
done
