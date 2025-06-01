# Europa @ 6th Kibo RPC -- Taiwanese Team

We are team **Europa** came from Taiwan, one of participating team of **the 6th Kibo Robot Programming Challenge**.

## Note
In order to make more fixiblity on DEIM ONNX model, we didn't push the model file `app/src/main/assets/DEIM.onnx`. **To make sure the model work in your local clone, please do the following steps**:

1. Train your own DEIM model by the original [ShihuaHuang95/DEIM](https://github.com/ShihuaHuang95/DEIM) repository by your custom dataset, and then convert the weight to ONNX model. Or choose one of our trained model from the [vision_models](https://github.com/Team-Europa/vision_models) repository.
2. Rename and move it as `app/src/main/assets/DEIM.onnx` in the Android Studio project.
3. Simplily build as the method Programming Manual mentioned.
