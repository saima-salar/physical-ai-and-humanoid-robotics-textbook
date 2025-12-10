const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-chatbot',

    getClientModules() {
      return [path.resolve(__dirname, './ChatbotInjector')];
    },
  };
};