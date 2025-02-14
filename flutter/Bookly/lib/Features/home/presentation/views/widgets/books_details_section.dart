import 'package:bookly/Features/home/presentation/views/widgets/books_action.dart';
import 'package:bookly/core/utils/styles.dart';
import 'package:flutter/material.dart';

import 'book_rating.dart';
import 'custom_book_image.dart';

class BookDetailsSection extends StatelessWidget {
  const BookDetailsSection({super.key});

  @override
  Widget build(BuildContext context) {
    var width = MediaQuery.of(context).size.width;
    return Column(
      children: [
        Padding(
          padding: EdgeInsets.symmetric(horizontal: width * 0.22),
          child: const CustomBookImage(imageUrl: 'https://www.futurepedia.io/_next/image?url=https%3A%2F%2Fcdn.sanity.io%2Fimages%2Fu0v1th4q%2Fproduction%2F0c6d1585cf097da3d56a40c7ae873e7c03a2724a-256x256.svg%3Fw%3D256%26h%3D256%26sharp%3D100%26q%3D100%26auto%3Dformat%26dpr%3D3&w=64&q=75',),
        ),
        const SizedBox(
          height: 42,
        ),
        Text(
          'The Jungle Book',
          style: Styles.textStyle30.copyWith(
            fontWeight: FontWeight.bold,
          ),
        ),
        const SizedBox(
          height: 6,
        ),
        Opacity(
          opacity: 0.7,
          child: Text(
            'Rudyard Kipling',
            style: Styles.textStyle18.copyWith(
              fontStyle: FontStyle.italic,
              fontWeight: FontWeight.w500,
            ),
          ),
        ),
        const BookRating(rating: 4, count: 400,),
        const SizedBox(
          height: 36,
        ),
        const BooksAction(),
      ],
    );
  }
}
