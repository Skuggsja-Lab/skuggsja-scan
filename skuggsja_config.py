import toml


class Config:
    def __init__(self, filename=None):
        self.resource_string = 'TCPIP::130.208.168.8::hislip0'
        self.traces_to_show = [1, 2, 3, 4]
        self.data_trace = 2
        self.max_x = 300
        self.min_x = 0
        self.VNA_settings = {}

        self.plot_settings = {"complex": True, "value_to_plot": "Mag dB"}

        with open("defaults.toml", mode="r") as fp:
            config = toml.load(fp)
            for attribute in config.keys():
                setattr(self, attribute, config[attribute])

        if filename:
            with open(filename, mode="r") as fp:
                config = toml.load(fp)
                for attribute in config.keys():
                    setattr(self, attribute, config[attribute])

    def save_toml(self, filename):
        with open(filename, mode="w") as fp:
            toml.dump(vars(self), fp)


# b = Config()
# b.save_toml("defaults.toml")
