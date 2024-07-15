from dataclasses import dataclass
import numpy as np
import functools
import operator
import logging
import time
from typing import List, Tuple
from pydrake.systems.framework import LeafSystem


class CurvatureSliders(LeafSystem):
    @dataclass
    class SliderDefault:
        """Default values for the meshcat sliders."""
        name: str
        """The name that is used to add / query values from."""
        default: float
        """The initial value of the slider."""

    _Q1 = SliderDefault("q1", 0.0)
    _Q2 = SliderDefault("q2", 0.0)

    def __init__(self, meshcat, plant, L=0.2):
        """
        @param meshcat The already created pydrake.geometry.Meshcat instance.
        @param plant The plant the sliders are connected to
        @param L the restlength of the segment
        """

        # Call Super Constructor
        LeafSystem.__init__(self)

        # Declare System Output
        output = self.DeclareVectorOutputPort(
            "q1_q2", 2, self.DoCalcOutput)
        output.disable_caching_by_default()

        # Init Class Variables
        self._meshcat = meshcat
        self._plant = plant
        self._L = L

        # Curvature Control Sliders
        self._meshcat.AddSlider(
            name=self._Q1.name, min=-2.0 * np.pi,
            max=2.0 * np.pi, step=0.1,
            value=self._Q1.default)
        self._meshcat.AddSlider(
            name=self._Q2.name, min=-2.0 * np.pi,
            max=2.0 * np.pi, step=0.1,
            value=self._Q2.default)

    def SetConfiguration(self, q: Tuple):
        """
        @param q configuration for each CC segment descriped by (q1, q2)
        """
        self._meshcat.SetSliderValue(self._Q1.name, q[0])
        self._meshcat.SetSliderValue(self._Q2.name, q[1])

    def CC2AGRB(self, q: Tuple) -> List[float]:

        # Extract input
        q1 = q[0]
        q2 = q[1]
        # Compute ARBM equivalent configuration
        config1 = [0, 0.5 * self._L, 0.5 * self._L, 0]
        config2 = [0, 0.5 * self._L, 0.5 * self._L, 0]

        if q1 != 0:
            config1 = [
                0.5 * q1,
                self._L * np.sin(0.5 * q1) / q1,
                self._L * np.sin(0.5 * q1) / q1,
                0.5 * q1
            ]
        if q2 != 0:
            config2 = [
                0.5 * q2,
                self._L * np.sin(0.5 * q2) / q2,
                self._L * np.sin(0.5 * q2) / q2,
                0.5 * q2
            ]

        return functools.reduce(operator.iconcat, [config1, config2], [])

    def DoCalcOutput(self, context, output):
        q1 = self._meshcat.GetSliderValue(self._Q1.name)
        q2 = self._meshcat.GetSliderValue(self._Q2.name)

        output.SetAtIndex(0, q1)
        output.SetAtIndex(1, q2)

    def Run(self, diagram, timeout=1e5):

        # Get all the contextes
        root_context = diagram.CreateDefaultContext()
        sliders_context = self.GetMyContextFromRoot(root_context)
        plant_context = self._plant.GetMyMutableContextFromRoot(root_context)

        # Add Stop Button
        kButtonName = "Stop Curvature Sliders"
        logging.info("Press the '{}' button in Meshcat to continue.",
                     kButtonName)
        self._meshcat.AddButton(kButtonName)

        # Greb current time to implement the timeout
        t0 = time.time()

        # Loop until the button is clicked, or
        # the timeout (when given) is reached.
        diagram.Publish(root_context)
        while self._meshcat.GetButtonClicks(kButtonName) < 1:
            # Break out of loop if timeout elapsed
            elapsed_t = time.time() - t0
            if elapsed_t >= timeout:
                break

            # If the sliders have not changed, avoid invalidating the context.
            old_positions = self._plant.GetPositions(plant_context)
            new_positions = self.CC2AGRB(
                self.get_output_port().Eval(sliders_context))
            if (np.abs(new_positions - old_positions) < 0.001).all():
                time.sleep(0.01)
                continue

            # Publish the new positions.
            self._plant.SetPositions(plant_context, new_positions)
            diagram.Publish(root_context)