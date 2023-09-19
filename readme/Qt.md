## Call `setValue` without trigerring `valueChangedSignal`

```python
slider = QSlider()
slider.blockSignals(True)
slider.setValue(20)
slider.blockSignals(False)
```