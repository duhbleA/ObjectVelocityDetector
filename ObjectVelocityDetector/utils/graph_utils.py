from object_detection.utils import label_map_util
"""
Utilities for loading frozen inference graphs and corresponding detection labels

"""


# Load the frozen graph from the file system and return the graph.
def load_model(tf, path_to_frozen_graph):
    """
    Loads a frozen serialized Tensor flow graph into Tensor Flow
    :param tf: The Tensor Flow session
    :param path_to_frozen_graph: Path to the frozen graph
    :return: A Tensor Flow graph with the loaded structures and weights of the serialized graph.
    """
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        obj_detect_graph_def = tf.GraphDef()
        with tf.gfile.GFile(path_to_frozen_graph, 'rb') as fid:
            serialized_graph = fid.read()
            obj_detect_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(obj_detect_graph_def, name="")
    return detection_graph


def get_categories(path_to_labels, max_num_classes):
    """
    Gets the colloquial categories from a path to a pbtext file and the number of classes
    :param path_to_labels: Plain text fully qualified path to pbtext file.
    :param max_num_classes: Max number of classes the pbtext file defines.
    :return: Categories for object detection.
    """
    label_map = label_map_util.load_labelmap(path_to_labels)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes, use_display_name=True)
    return categories


def get_category_indices(path_to_labels, max_num_classes):
    """
    Gets the category indices list from a pbtext file and the max number of classes.

    :param path_to_labels: Plain text fully qualified path to pbtext file.
    :param max_num_classes: Max number of classes the pbtext file defines.
    :return: A list of indices and corresponding labels for objects detected.
    """
    categories = get_categories(path_to_labels, max_num_classes)
    category_indices = label_map_util.create_category_index(categories)
    return category_indices
