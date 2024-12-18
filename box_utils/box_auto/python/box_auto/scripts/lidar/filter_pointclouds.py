from box_auto.utils import get_bag, upload_bag, run_ros_command

if __name__ == "__main__":
    livox_bag = get_bag("*nuc_livox.bag")
    hesai_bag = get_bag("*nuc_hesai_post_processed.bag")

    livox_out_bag = livox_bag.replace("nuc_livox.bag", "nuc_livox_filtered.bag")
    hesai_out_bag = hesai_bag.replace("nuc_hesai_post_processed.bag", "nuc_hesai_filtered.bag")
    run_ros_command(f"rosrun box_auto filter_pointclouds {livox_bag} {livox_out_bag} {hesai_bag} {hesai_out_bag}")

    upload_bag([livox_out_bag, hesai_out_bag])
