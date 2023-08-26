#ifndef BOXCARFILTER_H
#define BOXCARFILTER_H


class BoxcarFilter
{
    public:
        BoxcarFilter();
        virtual ~BoxcarFilter();

        void setLength(int _length);
        float addSample(float sample);

    protected:
    private:


    float history[100];
    int length;
    int currentPos;
    int currentLength;
    float total;
};

#endif // BOXCARFILTER_H
