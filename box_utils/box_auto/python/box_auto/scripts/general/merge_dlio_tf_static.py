from box_auto.utils import get_bag, upload_bag, run_ros_command, BOX_AUTO_SCRIPTS_DIR


if __name__ == "__main__":
    dlio_bag = get_bag("*_dlio.bag")
    tf_static_bag = get_bag("*_tf_static_start_end.bag")

    out_bag = dlio_bag.replace("_dlio.bag", "_tf_static_dlio_tf.bag")

    run_ros_command(
        f'python3 {BOX_AUTO_SCRIPTS_DIR}/general/merge_bags.py --input={dlio_bag},{tf_static_bag} --output={out_bag} --filter="/tf /tf_static"'
    )

    upload_bag(out_bag)
