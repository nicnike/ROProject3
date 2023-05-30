import unittest
from unittest.mock import MagicMock
from ..machine_learning.machineLearningNode import machineLearning, machineLearningNode

class machineLearningTests(unittest.TestCase):
    def test_ModelInitialization(self):
        '''!
        Tests if machine learning model could be loaded.
        '''
        ml = machineLearning()
        self.assertIsNotNone(ml.model)

    def test_PredictFunktion(self):
        '''!
        Tests predict function of the machine learning model for correct classification between cat and unicorn.
        '''
        ml = machineLearning()
        self.assertIsEqual(ml.prediction(radius=140, shape=5), 0)  # Cat
        self.assertIsEqual(ml.prediction(radius=155, shape=8), 1)  # Unicorn

    def test_MLNode(self):
        '''!
        Tests machine learning node.
        '''
        node = machineLearningNode()
        node.create_publisher = MagicMock()
        node.get_logger = MagicMock()
        msg = MagicMock()
        msg.shape = 8
        msg.radius = 155
        node.callback_classification(msg)
        node.publisher_.publish.assert_called_with(1)


