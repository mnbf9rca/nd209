
# Project: Follow Me
Robert Aleck / mnbf9rca
February 2019
## [Rubric](https://review.udacity.com/#!/rubrics/1155/view) points

### Network Architecture
The semantic segmentation model was built using a fully convolutional network (FCN). My initial model, based on the work I'd done in previous exercises, had two encoders, however this failed to reach the required level of accuracy. I added a third encoder/decoder pair, and experimented with the depth of filters for each layer - a sample of such values, the approximate training time (in minutes), and the resulting final score are shown below. 

I did note that although the course literature states that the number of filters should increase with each encoder layer (or decrease symmetrically for the decoder layers), I achieved the best result when encoder 2 and 3 were both set to 64 filters. You can find this model as file `model_weights_32_64_64_128_64_64_32`, along with some others, in the `data/weights` folder.

| encoder 1 | encoder 2 | encoder 3 | convolution | decoder 1 | decoder 2 | decoder 3 | training time | final_score
|---|---|---|---|---|---|---|---|---
| 32 | 64 | -- | 64 | -- | 64 | 32 | ~40 | 0.333
| **32** | **64** | **64** | **128** | **64** | **64** | **32** | **~40** | **0.419**
| 32 | 64 | 128 | 256 | 128 | 64 | 32 | ~45 |0.402
| 16 | 32 | 64 | 128 | 64 | 32 | 16 | ~26 | 0.395
| 32 | 64 | 128 | 128 | 128 | 64 | 32 | ~43 |0.389

The final model is shown below:
![network architecture](https://raw.githubusercontent.com/mnbf9rca/nd209/master/tensorflow_for_deep_learning/RoboND-DeepLearning-Project/images/final_model.png)
and is implemented as follows (this code is simplified for presentation):

    def fcn_model(inputs, num_classes):
	    encoder_layer_1 = encoder_block(inputs, filters = 32, strides = 2)
	    encoder_layer_2 = encoder_block(encoder_layer_1, filters = 64, strides = 2)
	    encoder_layer_3 = encoder_block(encoder_layer_2, filters = 64, strides = 2)
	    convolution_layer = conv2d_batchnorm(encoder_layer_3, filters = 128, kernel_size = 1, strides = 1)
	    decoder_layer_1 = decoder_block(convolution_layer, encoder_layer_2, filters = 64)
	    decoder_layer_2 = decoder_block(decoder_layer_1, encoder_layer_1, filters = 64)
	    decoder_layer_3 = decoder_block(decoder_layer_2, inputs, filters = 32)
	    x = decoder_layer_3
	    return layers.Conv2D(num_classes, 1, activation='softmax', padding='same')(x)

The **encoder** layers each consist of a separable convolution layer followed by a relu activation, and the output is then subject to batch normalisation to allow a higher learning rate by offsetting internal covariate shift caused by the change in network parameters which occurs during training. This allows us to train faster for a given learning rate.
The **1 x 1 convolution** is added between the encoder and decoder layers. This allows us to use images of any size, a reduction in dimensionality, and an increase in depth without a massive "explosion" in compute time caused by a massive increase in parameters.
The **decoder** layers have a single bilinear upsampling layer, a concatenation layer which concatenates the current layer with the 'skip' output of a previous layer thus restoring some of the spatial data lost by downsamping, and two separable convolution layers, which help extract the spatial data from the concatenated layer. 

### hyperparameter tuning
Various permutations of learning rate, batch size, and number of epochs were trialled. In general:

 - increasing **batch size** slowed training without significant impact on accuracy, and so was left at 64.
 - The **learning rate** was kept fairly low. At higher rates (0.01), the model tended to fail to converge.
 - in most models, the validation loss tended to improve only marginally above 4-6 **epochs**, and in some cases, final accuracy degraded when over 20 epochs (overfitting?). 10 epochs seems to be about right for this model, if a little on the high side. Each epoch takes approximately 5-10 minutes to train.
 - **Steps per epoch** and **validation steps** could be optimised by calculating it: dividing the number of training images with the batch size, however i didn't get time to do this.
 - I didnt adjust **workers** from the recommended value.
### training data
I used the provided training data. Looking at the scores, the highest error rates (false negative) are on the set where the target is in vision but far away. I attempted to correct for this by capturing additional sample data - flying the drone at altitude or at a distance, while the target walked within vision, but ultimately the process was unsuccessful - accuracy actually dropped when i added this data to the training set. Below, you can see a screen shot of additional data being captured:
![capturing more data](https://raw.githubusercontent.com/mnbf9rca/nd209/master/tensorflow_for_deep_learning/RoboND-DeepLearning-Project/images/capturing_additional_data.png)
## Why encode and then decode the image?
Ultimately, our goal is pixelwise prediction of the semantic value of an input image. Deep learning networks allow us to capture semantic/contextual information, but requires a relatively deep model to do so, and maintaining the size of input layer throughout would require large amounts of memory, and massive numbers of parameters to tune. By downsampling we can maintain the information needed to extract contextual information from the input, but at the cost of a loss of spatial resolution (as the data is transposed). We can restore that data by upsampling - effectively (but not totally) reversing the loss of spatial information. To fully recover the lost spatial information, we use skip connections, allowing us to concatenate the feature maps of the downsampled path withe the upsampled feature maps. 
## Is this model and data transferable to another context?
In short - no. The model has been trained on a specific "hero" who (it appears) is wearing a red coat when all other actors are in grey or blue. I would hypothesise that the model would fail should any other actors wear a red coat. The specific shape of the person is also unlikely to be transferrable to other contexts, such as a cat or cow. 
This model is not particularly complex, taking less than an hour to tune. If it were more generalised - for example, if it were trained to track a range of target objects by arbitrary selection (e.g. from imagenet), it is feasible that some form of transfer learning could be used to "seed" the initial downsampled model.
![drone following the hero](https://raw.githubusercontent.com/mnbf9rca/nd209/master/tensorflow_for_deep_learning/RoboND-DeepLearning-Project/images/following.png)
## Future enhancements
Future work might consider:
 - training for more general characteristics of the hero, such as gait, through more detailed analysis of the hero
 - Improve accuracy (IOU) through additional layers and further training
 - generate a larger training set of the hero at a distance, and train on this

> Written with [StackEdit](https://stackedit.io/).