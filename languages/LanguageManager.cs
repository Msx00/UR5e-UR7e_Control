using System;
using System.Linq;
using System.Windows;

namespace WpfRobot.languages
{
    public static class LanguageManager
    {
        private const string ZhPath = "Resources/languages/strings.zh-CN.xaml";
        private const string EnPath = "Resources/languages/strings.en-US.xaml";

        public static string CurrentLanguage { get; private set; } = "zh-CN";

        public static void ChangeLanguage(string language)
        {
            string path;

            if (language == "en-US")
            {
                path = EnPath;
                CurrentLanguage = "en-US";
            }
            else
            {
                path = ZhPath;
                CurrentLanguage = "zh-CN";
            }

            ResourceDictionary newDict = new ResourceDictionary
            {
                Source = new Uri(path, UriKind.Relative)
            };

            var dictionaries = System.Windows.Application.Current.Resources.MergedDictionaries;

            var oldDict = dictionaries.FirstOrDefault(d =>
                d.Source != null &&
                (
                    d.Source.OriginalString.Contains("Strings.zh-CN.xaml") ||
                    d.Source.OriginalString.Contains("Strings.en-US.xaml")
                )
            );

            if (oldDict != null)
            {
                dictionaries.Remove(oldDict);
            }

            dictionaries.Add(newDict);
        }

        public static string Get(string key)
        {
            object value = System.Windows.Application.Current.TryFindResource(key);

            if (value == null)
                return key;

            return value.ToString();
        }
    }
}